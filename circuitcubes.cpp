/*
 * circuitcubes.cpp - bare-metal BLE Central for Circuit Cubes (micro:bit v2).
 *
 * Drives NRF_RADIO directly (1M PHY, legacy advertising + data channels):
 *   scan ch37/38/39 -> match ADV_IND local name -> CONNECT_IND (T_IFS later)
 *   -> master connection events -> minimal LL control replies
 *   -> ATT Read By Type (0x2803) to find the Nordic UART write characteristic
 *   -> ATT Write Command (0x52) / Write Request (0x12) with motor strings.
 *
 * Resources taken over (see the #defines below, change them if CODAL on your
 * target version already uses them):
 *   - NRF_RADIO + RADIO_IRQn
 *   - NRF_TIMER0 (free running, 1 MHz) + TIMER0_IRQn
 *   - PPI channels 18 and 19
 *
 * Do NOT combine with the MakeCode "bluetooth" or "radio" extensions.
 */
#include "pxt.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include <string.h>

// CODAL v2 uses TIMER1 (system), TIMER2 (ADC), TIMER3 (touch), TIMER4 (display)
// and RADIO_IRQHandler (MicroBitRadio.cpp). TIMER0 belongs to the SoftDevice,
// which we disable, so it is free. Our ISRs are installed at runtime with
// NVIC_SetVector (CODAL keeps the vector table in RAM) to avoid a duplicate
// RADIO_IRQHandler symbol.
#define CC_TIMER            NRF_TIMER0
#define CC_TIMER_IRQn       TIMER0_IRQn
#define CC_PPI_TX           18      // TIMER compare0 -> RADIO TXEN
#define CC_PPI_TS           19      // RADIO END      -> TIMER capture2

// 1 = "+000a" for stop (always 5 bytes), 0 = "000a" (4 bytes)
#define CC_SIGNED_STOP      1

#define R   NRF_RADIO
#define T   CC_TIMER

namespace circuitcubes {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static const uint32_t ADV_AA        = 0x8E89BED6;
static const uint32_t ADV_CRCINIT   = 0x555555;

static const uint32_t TX_RAMP_US    = 41;     // TXEN -> first preamble bit (fast ramp-up)
static const uint32_t PREP_LEAD_US  = 600;    // prepare event this long before anchor
static const uint32_t SCAN_DWELL_US = 25000;  // time per advertising channel

static const uint32_t CONN_INTERVAL_UNITS = 24;     // 30 ms
static const uint32_t CONN_INTERVAL_US    = CONN_INTERVAL_UNITS * 1250;
static const uint32_t CONN_TIMEOUT_UNITS  = 200;    // 2 s
static const uint32_t CONN_TIMEOUT_US     = CONN_TIMEOUT_UNITS * 10000;
static const uint32_t CONN_ESTABLISH_US   = 6 * CONN_INTERVAL_US;
static const uint8_t  CONN_HOP            = 7;
static const uint8_t  CONN_WIN_SIZE       = 2;      // x1.25 ms
static const uint8_t  CONN_WIN_OFFSET     = 1;      // x1.25 ms

// Nordic UART Service TX/"write" characteristic 6e400002-b5a3-f393-e0a9-e50e24dcca9e (LE)
static const uint8_t NUS_WRITE_UUID[16] = {
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e };

// (plain constants, not enums: pxt would otherwise export C++ enums to TypeScript)
typedef uint8_t State;
typedef uint8_t Phase;
static const State ST_OFF = 0, ST_SCAN = 1, ST_INIT_TX = 2, ST_CONN = 3,
                   ST_SCAN_REQ = 4, ST_SCAN_RSP = 5;
static const Phase PH_NONE = 0, PH_TX = 1, PH_RX = 2;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static bool g_inited = false;
static volatile State g_state = ST_OFF;
static volatile Phase g_phase = PH_NONE;
static volatile bool g_ready = false;           // write characteristic known

static uint8_t rxbuf[72] __attribute__((aligned(4)));
static uint8_t txdma[72] __attribute__((aligned(4)));
static uint8_t my_addr[6];
static uint32_t rng_state;

// debug counters (see debugStat): 1 adv rx ok, 2 adv crc bad, 3 ADV_IND, 4 SCAN_RSP,
// 5 name match, 6 CONNECT_IND sent, 7 conn events answered, 8 conn events missed, 9 disconnects
static volatile uint32_t stat[12];   // 10 = SCAN_REQ sent
static char    last_name[2][32];          // [0] from ADV_IND, [1] from SCAN_RSP
static uint8_t last_name_len[2];

// active scanning
static uint8_t target_addr[6];
static bool    have_target;
static uint8_t cur_adva[6];
static volatile bool rsp_end, rsp_to;

// scanning
static const uint8_t ADV_CH[3] = { 37, 38, 39 };
static uint8_t scan_idx;
static volatile bool scan_rot;

// connection
static uint32_t conn_aa, conn_crc;
static uint32_t anchor;             // next event anchor (timer us)
static uint32_t last_rx;            // last valid rx (timer us)
static uint32_t established_deadline;
static bool     established;
static uint8_t  chan_unmapped;
static uint8_t  tx_sn, rx_nesn;
static volatile bool rx_end_flag, rx_to, rx_crc_ok;
static bool     disconnect_req;
static bool     version_sent;

// pending LL tx
static bool     tx_has;
static uint8_t  tx_llid, tx_len;
static uint8_t  tx_payload[32];
static uint8_t  ctrl_buf[16];
static uint8_t  ctrl_len;

// ATT / GATT
static uint8_t  att_req[8];
static uint8_t  att_req_len;
static bool     att_inflight;
static uint16_t gatt_handle;        // NUS write characteristic value handle
static uint8_t  gatt_props;
static bool     att_wait_rsp;
static uint32_t att_wait_until;

// application -> ISR
static volatile int16_t g_speed[3];
static volatile uint8_t g_dirty;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
static inline uint32_t tnow() {
    T->TASKS_CAPTURE[4] = 1;
    return T->CC[4];
}
static inline bool in_future(uint32_t t, uint32_t margin_us) {
    return (int32_t)(t - tnow()) > (int32_t)margin_us;
}
static inline void clr(volatile uint32_t *ev) {
    *ev = 0;
    (void)*ev;
}
static uint32_t rnd() {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static bool aa_valid(uint32_t aa) {
    if (aa == ADV_AA) return false;
    if (__builtin_popcount(aa ^ ADV_AA) <= 1) return false;
    uint8_t b0 = aa & 0xFF;
    if (b0 == ((aa >> 8) & 0xFF) && b0 == ((aa >> 16) & 0xFF) && b0 == (aa >> 24)) return false;
    int run = 1, trans = 0;
    for (int i = 1; i < 32; i++) {
        if (((aa >> i) & 1) == ((aa >> (i - 1)) & 1)) {
            if (++run > 6) return false;
        } else {
            run = 1;
            trans++;
        }
    }
    if (trans > 24) return false;
    int t6 = 0;
    for (int i = 26; i < 31; i++)
        if (((aa >> i) & 1) != ((aa >> (i + 1)) & 1)) t6++;
    return t6 >= 2;
}

static uint8_t freq_for(uint8_t ch) {
    if (ch == 37) return 2;
    if (ch == 38) return 26;
    if (ch == 39) return 80;
    return ch <= 10 ? 4 + 2 * ch : 6 + 2 * ch;
}
static void set_channel(uint8_t ch) {
    R->FREQUENCY = freq_for(ch);
    R->DATAWHITEIV = ch & 0x3F;
}
static void set_aa_crc(uint32_t aa, uint32_t crcinit) {
    R->BASE0 = (aa << 8) & 0xFFFFFF00UL;
    R->PREFIX0 = aa >> 24;
    R->CRCINIT = crcinit;
}

// ---------------------------------------------------------------------------
// SoftDevice / clocks / peripherals
// ---------------------------------------------------------------------------
static void disable_softdevice() {
    // Only touch the SD if one is actually flashed (SVCs would fault otherwise).
    const uint32_t SD_MAGIC = 0x51B1E5DB;
    if (*(volatile uint32_t *)(SOFTDEVICE_INFO_STRUCT_ADDRESS + 4) == SD_MAGIC) {
        uint8_t en = 0;
        if (sd_softdevice_is_enabled(&en) == NRF_SUCCESS && en)
            sd_softdevice_disable();
    }
}

static void start_hfclk() {
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    for (volatile uint32_t i = 0; i < 2000000 && !NRF_CLOCK->EVENTS_HFCLKSTARTED; i++) {}
}

static void radio_init() {
    R->TASKS_DISABLE = 1;
    for (volatile int i = 0; i < 10000 && (R->STATE & 0xF); i++) {}
    R->POWER = 0;
    R->POWER = 1;

    R->MODE = RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos;
    R->TXPOWER = RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos;
    R->MODECNF0 = (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos) |
                  (RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos);

    R->PCNF0 = (8 << RADIO_PCNF0_LFLEN_Pos) | (1 << RADIO_PCNF0_S0LEN_Pos) |
               (0 << RADIO_PCNF0_S1LEN_Pos) | (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);
    R->PCNF1 = (64 << RADIO_PCNF1_MAXLEN_Pos) | (0 << RADIO_PCNF1_STATLEN_Pos) |
               (3 << RADIO_PCNF1_BALEN_Pos) |
               (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
               (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

    R->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    R->CRCPOLY = 0x00065B;

    R->TXADDRESS = 0;
    R->RXADDRESSES = 1;
    set_aa_crc(ADV_AA, ADV_CRCINIT);
}

static void timer_init() {
    T->TASKS_STOP = 1;
    T->TASKS_CLEAR = 1;
    T->MODE = TIMER_MODE_MODE_Timer;
    T->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    T->PRESCALER = 4;                       // 16 MHz / 2^4 = 1 MHz
    T->INTENCLR = 0xFFFFFFFF;
    T->TASKS_START = 1;
}

static void ppi_init() {
    NRF_PPI->CH[CC_PPI_TS].EEP = (uint32_t)&R->EVENTS_END;
    NRF_PPI->CH[CC_PPI_TS].TEP = (uint32_t)&T->TASKS_CAPTURE[2];
    NRF_PPI->CH[CC_PPI_TX].EEP = (uint32_t)&T->EVENTS_COMPARE[0];
    NRF_PPI->CH[CC_PPI_TX].TEP = (uint32_t)&R->TASKS_TXEN;
    NRF_PPI->CHENSET = 1UL << CC_PPI_TS;
}
static inline void tx_trigger_on()  { NRF_PPI->CHENSET = 1UL << CC_PPI_TX; }
static inline void tx_trigger_off() { NRF_PPI->CHENCLR = 1UL << CC_PPI_TX; }

static void radio_force_disable() {
    R->SHORTS = 0;
    tx_trigger_off();
    R->TASKS_DISABLE = 1;
    for (volatile int i = 0; i < 5000 && (R->STATE & 0xF); i++) {}
    clr(&R->EVENTS_END);
    clr(&R->EVENTS_DISABLED);
    clr(&R->EVENTS_ADDRESS);
}

// ---------------------------------------------------------------------------
// Scanner / initiator
// ---------------------------------------------------------------------------
static void scan_start() {
    radio_force_disable();
    T->INTENCLR = TIMER_INTENCLR_COMPARE3_Msk;
    g_phase = PH_NONE;
    g_ready = false;
    have_target = false;
    g_state = ST_SCAN;
    scan_idx = 0;
    scan_rot = false;
    set_aa_crc(ADV_AA, ADV_CRCINIT);
    set_channel(ADV_CH[scan_idx]);
    R->PACKETPTR = (uint32_t)rxbuf;
    R->SHORTS = RADIO_SHORTS_READY_START_Msk;
    clr(&T->EVENTS_COMPARE[1]);
    T->CC[1] = tnow() + SCAN_DWELL_US;
    T->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
    R->INTENSET = RADIO_INTENSET_END_Msk | RADIO_INTENSET_DISABLED_Msk;
    R->TASKS_RXEN = 1;
}

static bool name_matches(const uint8_t *ad, uint8_t n) {
    static const char *names[2] = { "Tenka", "Circuit Cube" };
    uint8_t i = 0;
    while (i + 1 < n) {
        uint8_t l = ad[i];
        if (l == 0 || i + 1 + l > n) break;
        uint8_t type = ad[i + 1];
        const uint8_t *d = &ad[i + 2];
        uint8_t dl = l - 1;
        if (type == 0x08 || type == 0x09) {
            for (int k = 0; k < 2; k++) {
                uint8_t tl = strlen(names[k]);
                if (dl >= tl && memcmp(d, names[k], tl) == 0) return true;
                // shortened name: advertised text is a prefix of the target
                if (type == 0x08 && dl >= 3 && dl < tl && memcmp(d, names[k], dl) == 0) return true;
            }
        }
        i += l + 1;
    }
    return false;
}

static void build_connect_ind(const uint8_t *adv) {
    // new connection parameters
    do { conn_aa = rnd(); } while (!aa_valid(conn_aa));
    conn_crc = rnd() & 0xFFFFFF;

    uint8_t *p = txdma;
    p[0] = 0x05 | (1 << 6) | (((adv[0] >> 6) & 1) << 7);   // CONNECT_IND, TxAdd=random
    p[1] = 34;
    memcpy(p + 2, my_addr, 6);
    memcpy(p + 8, adv + 2, 6);
    uint8_t *ll = p + 14;
    ll[0] = conn_aa; ll[1] = conn_aa >> 8; ll[2] = conn_aa >> 16; ll[3] = conn_aa >> 24;
    ll[4] = conn_crc; ll[5] = conn_crc >> 8; ll[6] = conn_crc >> 16;
    ll[7] = CONN_WIN_SIZE;
    ll[8] = CONN_WIN_OFFSET; ll[9] = 0;
    ll[10] = CONN_INTERVAL_UNITS; ll[11] = 0;
    ll[12] = 0; ll[13] = 0;                                 // latency
    ll[14] = CONN_TIMEOUT_UNITS; ll[15] = 0;
    ll[16] = 0xFF; ll[17] = 0xFF; ll[18] = 0xFF; ll[19] = 0xFF; ll[20] = 0x1F;
    ll[21] = CONN_HOP | (0 << 5);                           // SCA 251-500 ppm (conservative)
}

static bool find_name(const uint8_t *ad, uint8_t n, const uint8_t **out, uint8_t *outl) {
    uint8_t i = 0;
    while (i + 1 < n) {
        uint8_t l = ad[i];
        if (l == 0 || i + 1 + l > n) break;
        if (ad[i + 1] == 0x08 || ad[i + 1] == 0x09) {
            *out = &ad[i + 2];
            *outl = l - 1;
            return true;
        }
        i += l + 1;
    }
    return false;
}

static void remember_name(const uint8_t *ad, uint8_t n, int kind) {
    const uint8_t *nm;
    uint8_t nl;
    if (!find_name(ad, n, &nm, &nl)) return;
    if (nl > 31) nl = 31;
    memcpy(last_name[kind], nm, nl);
    last_name_len[kind] = nl;
}

// Transmit txdma T_IFS after the packet that ended at tend (timer us). Returns false if too late.
static bool schedule_tx(uint32_t tend, uint32_t shorts, bool then_rx) {
    uint32_t at = tend + 150 - TX_RAMP_US;
    if (!in_future(at, 8)) return false;
    R->PACKETPTR = (uint32_t)txdma;
    R->SHORTS = shorts;
    T->CC[0] = at;
    clr(&T->EVENTS_COMPARE[0]);
    tx_trigger_on();
    R->TASKS_DISABLE = 1;
    if (then_rx) {
        // The DISABLED->RXEN short must not be armed while *this* disable is still in
        // flight, or it would fire immediately and the radio would sit in RX instead of TX.
        for (volatile int i = 0; i < 400 && (R->STATE & 0xF); i++) {}
        R->SHORTS = shorts | RADIO_SHORTS_DISABLED_RXEN_Msk;
    }
    return true;
}

static void build_scan_req(const uint8_t *adv) {
    txdma[0] = 0x03 | (1 << 6) | (((adv[0] >> 6) & 1) << 7);   // SCAN_REQ, TxAdd=random
    txdma[1] = 12;
    memcpy(txdma + 2, my_addr, 6);
    memcpy(txdma + 8, adv + 2, 6);
    memcpy(cur_adva, adv + 2, 6);
}

static void scan_rx_end() {
    uint32_t tend = T->CC[2];
    if (!(R->CRCSTATUS & 1)) {
        stat[2]++;
        R->TASKS_START = 1;
        return;
    }
    stat[1]++;
    uint8_t type = rxbuf[0] & 0x0F;
    uint8_t len = rxbuf[1] & 0x3F;
    if ((type == 0 || type == 6) && len >= 6 && len <= 37) {
        const uint8_t *adva = &rxbuf[2];
        if (type == 0) stat[3]++;
        remember_name(&rxbuf[8], len - 6, 0);
        bool match = name_matches(&rxbuf[8], len - 6) ||
                     (have_target && memcmp(adva, target_addr, 6) == 0);
        if (match && type == 0) {
            stat[5]++;
            build_connect_ind(rxbuf);
            if (schedule_tx(tend, RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk, false)) {
                stat[6]++;
                T->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
                g_state = ST_INIT_TX;
                return;
            }
        } else if (!match) {
            // name may be in the scan response: ask for it (active scan, like a phone does)
            stat[10]++;
            build_scan_req(rxbuf);
            if (schedule_tx(tend, RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk, true)) {
                g_state = ST_SCAN_REQ;
                return;
            }
        }
    }
    R->TASKS_START = 1;     // radio is in RXIDLE: keep listening
}

static void scan_req_tx_end() {
    uint32_t tend = T->CC[2];
    tx_trigger_off();
    R->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;   // no more RXEN
    R->PACKETPTR = (uint32_t)rxbuf;
    clr(&R->EVENTS_ADDRESS);
    clr(&R->EVENTS_END);
    rsp_end = false;
    rsp_to = false;
    T->CC[3] = tend + 400;
    clr(&T->EVENTS_COMPARE[3]);
    T->INTENSET = TIMER_INTENSET_COMPARE3_Msk;
    g_state = ST_SCAN_RSP;
}

static void scan_rsp_end() {
    rsp_end = true;
    if (!(R->CRCSTATUS & 1)) return;
    uint8_t len = rxbuf[1] & 0x3F;
    if ((rxbuf[0] & 0x0F) != 4 || len < 6 || len > 37) return;
    if (memcmp(&rxbuf[2], cur_adva, 6) != 0) return;
    stat[4]++;
    remember_name(&rxbuf[8], len - 6, 1);
    if (name_matches(&rxbuf[8], len - 6)) {
        memcpy(target_addr, cur_adva, 6);
        have_target = true;     // connect on its next ADV_IND
    }
}

static void resume_scan() {
    T->INTENCLR = TIMER_INTENCLR_COMPARE3_Msk;
    g_state = ST_SCAN;
    R->SHORTS = RADIO_SHORTS_READY_START_Msk;
    R->PACKETPTR = (uint32_t)rxbuf;
    R->TASKS_RXEN = 1;
}

// ---------------------------------------------------------------------------
// Connection (master side)
// ---------------------------------------------------------------------------
static void go_scan() {
    stat[9]++;
    scan_start();
}

static void att_queue(const uint8_t *p, uint8_t n) {
    memcpy(att_req, p, n);
    att_req_len = n;
}

static void att_request_discovery(uint16_t start) {
    uint8_t rq[7] = { 0x08, (uint8_t)start, (uint8_t)(start >> 8), 0xFF, 0xFF, 0x03, 0x28 };
    att_queue(rq, 7);
    att_inflight = false;
}

static void conn_begin() {
    uint32_t tend = T->CC[2];
    tx_trigger_off();
    anchor = tend + 1250 + 1250 * CONN_WIN_OFFSET + 400;
    last_rx = tend;
    established = false;
    established_deadline = tend + CONN_ESTABLISH_US + 10000;
    chan_unmapped = 0;
    tx_sn = rx_nesn = 0;
    tx_has = false;
    ctrl_len = 0;
    disconnect_req = false;
    version_sent = false;
    att_wait_rsp = false;
    g_ready = false;
    gatt_handle = 0;
    g_dirty = 0;
    att_request_discovery(0x0001);

    set_aa_crc(conn_aa, conn_crc);
    R->SHORTS = 0;
    g_phase = PH_NONE;
    g_state = ST_CONN;
    clr(&T->EVENTS_COMPARE[1]);
    T->CC[1] = anchor - PREP_LEAD_US;
    T->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
}

static void wrap_att(const uint8_t *att, uint8_t n) {
    tx_payload[0] = n; tx_payload[1] = 0; tx_payload[2] = 4; tx_payload[3] = 0;
    memcpy(tx_payload + 4, att, n);
    tx_len = n + 4;
    tx_llid = 2;
    tx_has = true;
}

static void fetch_tx() {
    if (ctrl_len) {
        memcpy(tx_payload, ctrl_buf, ctrl_len);
        tx_len = ctrl_len;
        tx_llid = 3;
        tx_has = true;
        ctrl_len = 0;
        return;
    }
    if (att_req_len && !att_inflight) {
        wrap_att(att_req, att_req_len);
        att_req_len = 0;
        att_inflight = true;
        return;
    }
    if (att_wait_rsp && (int32_t)(tnow() - att_wait_until) > 0) att_wait_rsp = false;
    if (g_ready && g_dirty && !att_wait_rsp) {
        uint8_t dirty = __atomic_exchange_n(&g_dirty, 0, __ATOMIC_ACQ_REL);
        uint8_t att[3 + 3 * 5];
        uint8_t n = 0;
        bool no_rsp = gatt_props & 0x04;
        att[n++] = no_rsp ? 0x52 : 0x12;
        att[n++] = gatt_handle;
        att[n++] = gatt_handle >> 8;
        for (int i = 0; i < 3; i++) {
            if (!(dirty & (1 << i))) continue;
            int s = g_speed[i];
            char sign = s < 0 ? '-' : '+';
            if (s < 0) s = -s;
            if (s > 255) s = 255;
#if !CC_SIGNED_STOP
            if (s == 0) {
                att[n++] = '0'; att[n++] = '0'; att[n++] = '0';
                att[n++] = 'a' + i;
                continue;
            }
#endif
            att[n++] = sign;
            att[n++] = '0' + s / 100;
            att[n++] = '0' + (s / 10) % 10;
            att[n++] = '0' + s % 10;
            att[n++] = 'a' + i;
        }
        wrap_att(att, n);
        if (!no_rsp) {
            att_wait_rsp = true;
            att_wait_until = tnow() + 1000000;
        }
    }
}

static void ctrl_reply(const uint8_t *p, uint8_t n) {
    if (ctrl_len) return;
    memcpy(ctrl_buf, p, n);
    ctrl_len = n;
}

static void handle_ll_control(const uint8_t *d, uint8_t len) {
    if (len < 1) return;
    uint8_t op = d[0];
    switch (op) {
    case 0x02:  // LL_TERMINATE_IND
        disconnect_req = true;
        break;
    case 0x08:  // LL_FEATURE_REQ
    case 0x0E: {// LL_SLAVE_FEATURE_REQ
        uint8_t r[9] = { 0x09, 0, 0, 0, 0, 0, 0, 0, 0 };
        ctrl_reply(r, 9);
        break; }
    case 0x0C:  // LL_VERSION_IND
        if (!version_sent) {
            uint8_t r[6] = { 0x0C, 0x09, 0xFF, 0xFF, 0x00, 0x00 };
            ctrl_reply(r, 6);
            version_sent = true;
        }
        break;
    case 0x12: {// LL_PING_REQ
        uint8_t r[1] = { 0x13 };
        ctrl_reply(r, 1);
        break; }
    case 0x07: case 0x09: case 0x13: case 0x11:
        break;  // responses: ignore
    default: {
        uint8_t r[2] = { 0x07, op };            // LL_UNKNOWN_RSP
        ctrl_reply(r, 2);
        break; }
    }
}

static void att_rx(const uint8_t *p, uint8_t n) {
    if (n < 1) return;
    switch (p[0]) {
    case 0x09: {    // Read By Type Response
        if (n < 2) break;
        uint8_t il = p[1];
        uint16_t last = 0;
        att_inflight = false;
        for (uint8_t i = 2; il >= 7 && i + il <= n; i += il) {
            last = p[i] | (p[i + 1] << 8);
            if (il == 21 && memcmp(&p[i + 5], NUS_WRITE_UUID, 16) == 0) {
                gatt_props = p[i + 2];
                gatt_handle = p[i + 3] | (p[i + 4] << 8);
                g_ready = true;
                return;
            }
        }
        if (last && last < 0xFFFF) att_request_discovery(last + 1);
        else disconnect_req = true;     // reached end: not a Circuit Cube
        break; }
    case 0x01:      // Error Response
        if (n >= 2 && p[1] == 0x08) {
            att_inflight = false;
            if (!g_ready) disconnect_req = true;    // Attribute Not Found: no NUS write char
        } else if (n >= 2 && (p[1] == 0x12 || p[1] == 0x52)) {
            att_wait_rsp = false;
        }
        break;
    case 0x13:      // Write Response
        att_wait_rsp = false;
        break;
    default:
        break;      // notifications, etc.: ignored
    }
}

static void process_rx() {
    uint8_t hdr = rxbuf[0];
    uint8_t len = rxbuf[1];
    uint8_t llid = hdr & 3;
    uint8_t nesn = (hdr >> 2) & 1;
    uint8_t sn = (hdr >> 3) & 1;
    if (len > 62) return;

    if (nesn != tx_sn) {            // peer acknowledged our last PDU
        tx_sn ^= 1;
        tx_has = false;
    }
    if (sn == rx_nesn) {            // new data from peer
        rx_nesn ^= 1;
        const uint8_t *d = &rxbuf[2];
        if (llid == 3) {
            handle_ll_control(d, len);
        } else if ((llid == 2 || llid == 1) && len >= 4) {
            uint16_t l2len = d[0] | (d[1] << 8);
            uint16_t cid = d[2] | (d[3] << 8);
            if (cid == 4 && l2len + 4 <= len) att_rx(d + 4, l2len);
        }
    }
}

static void finish_event(bool got) {
    g_phase = PH_NONE;
    T->INTENCLR = TIMER_INTENCLR_COMPARE3_Msk;
    uint32_t now = tnow();
    if (got) {
        stat[7]++;
        process_rx();
        last_rx = now;
        established = true;
    }
    if (!got) stat[8]++;
    uint32_t limit = established ? CONN_TIMEOUT_US : CONN_ESTABLISH_US;
    if (disconnect_req || (int32_t)(now - last_rx) > (int32_t)limit) go_scan();
}

static void conn_prepare() {
    if (R->STATE & 0xF) radio_force_disable();
    uint32_t at = anchor - TX_RAMP_US;
    uint32_t this_anchor = anchor;
    anchor += CONN_INTERVAL_US;
    T->CC[1] = anchor - PREP_LEAD_US;
    if (!in_future(at, 15)) return;             // missed this event

    chan_unmapped = (chan_unmapped + CONN_HOP) % 37;
    set_channel(chan_unmapped);

    if (!tx_has) fetch_tx();
    txdma[0] = (tx_has ? tx_llid : 1) | (rx_nesn << 2) | (tx_sn << 3);
    txdma[1] = tx_has ? tx_len : 0;
    if (tx_has) memcpy(&txdma[2], tx_payload, tx_len);

    rx_end_flag = false;
    rx_to = false;
    clr(&R->EVENTS_END);
    clr(&R->EVENTS_DISABLED);
    clr(&R->EVENTS_ADDRESS);
    R->PACKETPTR = (uint32_t)txdma;
    R->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |
                RADIO_SHORTS_DISABLED_RXEN_Msk;
    g_phase = PH_TX;
    T->CC[0] = at;
    clr(&T->EVENTS_COMPARE[0]);
    tx_trigger_on();
    (void)this_anchor;
}

static void conn_tx_end() {
    uint32_t tend = T->CC[2];
    tx_trigger_off();
    R->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;   // no more RXEN
    R->PACKETPTR = (uint32_t)rxbuf;
    clr(&R->EVENTS_ADDRESS);
    clr(&R->EVENTS_END);
    g_phase = PH_RX;
    T->CC[3] = tend + 400;      // response must have started by then
    clr(&T->EVENTS_COMPARE[3]);
    T->INTENSET = TIMER_INTENSET_COMPARE3_Msk;
}

// ---------------------------------------------------------------------------
// Interrupt dispatch
// ---------------------------------------------------------------------------
static void on_radio_end() {
    switch (g_state) {
    case ST_SCAN:
        scan_rx_end();
        break;
    case ST_SCAN_REQ:
        scan_req_tx_end();
        break;
    case ST_SCAN_RSP:
        scan_rsp_end();
        break;
    case ST_INIT_TX:
        conn_begin();
        break;
    case ST_CONN:
        if (g_phase == PH_TX) {
            conn_tx_end();
        } else if (g_phase == PH_RX) {
            rx_crc_ok = R->CRCSTATUS & 1;
            rx_end_flag = true;
        }
        break;
    default:
        break;
    }
}

static void on_radio_disabled() {
    switch (g_state) {
    case ST_SCAN:
        if (scan_rot) {
            scan_rot = false;
            scan_idx = (scan_idx + 1) % 3;
            set_channel(ADV_CH[scan_idx]);
            R->TASKS_RXEN = 1;
        }
        break;
    case ST_SCAN_RSP:
        if (rsp_end || rsp_to) resume_scan();
        break;
    case ST_CONN:
        if (g_phase == PH_RX && (rx_end_flag || rx_to))
            finish_event(rx_end_flag && rx_crc_ok);
        break;
    default:
        break;
    }
}

} // namespace circuitcubes

using namespace circuitcubes;

static void cc_radio_isr(void) {
    if (R->EVENTS_END) {
        clr(&R->EVENTS_END);
        on_radio_end();
    }
    if (R->EVENTS_DISABLED) {
        clr(&R->EVENTS_DISABLED);
        on_radio_disabled();
    }
}

static void cc_timer_isr(void) {
    if (T->EVENTS_COMPARE[1]) {
        clr(&T->EVENTS_COMPARE[1]);
        if (g_state == ST_CONN) {
            conn_prepare();
        } else {
            T->CC[1] = tnow() + SCAN_DWELL_US;
            if (g_state == ST_SCAN) {
                scan_rot = true;
                R->TASKS_DISABLE = 1;
            }
        }
    }
    if (T->EVENTS_COMPARE[3]) {
        clr(&T->EVENTS_COMPARE[3]);
        T->INTENCLR = TIMER_INTENCLR_COMPARE3_Msk;
        if (g_state == ST_CONN && g_phase == PH_RX && !R->EVENTS_ADDRESS) {
            rx_to = true;
            R->SHORTS = 0;
            R->TASKS_DISABLE = 1;
        } else if (g_state == ST_SCAN_RSP && !R->EVENTS_ADDRESS) {
            rsp_to = true;
            R->SHORTS = 0;
            R->TASKS_DISABLE = 1;
        }
    }
}

// ---------------------------------------------------------------------------
// MakeCode shims
// ---------------------------------------------------------------------------
namespace circuitcubes {

static void stack_init() {
    disable_softdevice();
    start_hfclk();

    my_addr[0] = NRF_FICR->DEVICEADDR[0];
    my_addr[1] = NRF_FICR->DEVICEADDR[0] >> 8;
    my_addr[2] = NRF_FICR->DEVICEADDR[0] >> 16;
    my_addr[3] = NRF_FICR->DEVICEADDR[0] >> 24;
    my_addr[4] = NRF_FICR->DEVICEADDR[1];
    my_addr[5] = (NRF_FICR->DEVICEADDR[1] >> 8) | 0xC0;     // random static
    rng_state = NRF_FICR->DEVICEID[0] ^ (NRF_FICR->DEVICEID[1] << 7) ^ 0x9E3779B9;
    if (!rng_state) rng_state = 1;

    radio_init();
    timer_init();
    ppi_init();
    rng_state ^= tnow();

    NVIC_SetVector(RADIO_IRQn, (uint32_t)cc_radio_isr);
    NVIC_SetVector(CC_TIMER_IRQn, (uint32_t)cc_timer_isr);
    NVIC_SetPriority(RADIO_IRQn, 0);
    NVIC_SetPriority(CC_TIMER_IRQn, 1);
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_ClearPendingIRQ(CC_TIMER_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(CC_TIMER_IRQn);
    g_inited = true;
}

//%
void startConnect() {
    if (!g_inited) stack_init();
    if (g_state == ST_OFF) scan_start();
}

//%
bool isConnected() {
    return g_inited && g_state == ST_CONN && g_ready;
}

//%
void setMotorRaw(int port, int speed) {
    if (port < 1 || port > 3 || !isConnected()) return;
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    g_speed[port - 1] = speed;
    __atomic_fetch_or(&g_dirty, 1 << (port - 1), __ATOMIC_ACQ_REL);
}

//%
int debugStat(int i) {
    if (i == 0) return g_state;
    if (i < 1 || i > 10) return 0;
    return (int)stat[i];
}

//%
String debugName(int kind) {
    if (kind < 0 || kind > 1) kind = 0;
    return mkString(last_name[kind], last_name_len[kind]);
}

//%
void stopAll() {
    if (!isConnected()) return;
    for (int i = 0; i < 3; i++) g_speed[i] = 0;
    __atomic_fetch_or(&g_dirty, 7, __ATOMIC_ACQ_REL);
}

} // namespace circuitcubes

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
static const uint32_t PREP_LEAD_US  = 1000;   // prepare event this long before anchor (ISR latency budget)
static const uint32_t PREP_MIN_US   = 40;     // less time than this left before TX: skip the event
static const uint32_t SCAN_DWELL_US = 25000;  // time per advertising channel

static const uint16_t CONN_INTERVAL_UNITS = 24;     // 30 ms
static const uint32_t CONN_INTERVAL_US    = CONN_INTERVAL_UNITS * 1250;
static const uint16_t CONN_TIMEOUT_UNITS  = 400;    // 4 s
static const uint32_t CONN_ESTABLISH_US   = 6 * CONN_INTERVAL_US;
static const uint8_t  CONN_HOP            = 7;
static const uint8_t  CONN_WIN_SIZE       = 2;      // x1.25 ms
static const uint8_t  CONN_WIN_OFFSET     = 1;      // x1.25 ms
static const uint16_t UPD_INSTANT_EVENTS  = 10;     // LL_CONNECTION_UPDATE_IND instant, events ahead

// why the last connection ended (debugStat 24)
static const uint8_t CAUSE_TIMEOUT = 1, CAUSE_ESTABLISH = 2, CAUSE_PEER = 3, CAUSE_NOT_CUBE = 4,
                     CAUSE_USER = 5;

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
static volatile uint32_t stat[40];   // 18/19: setMotor calls / accepted   // 10 = SCAN_REQ sent
// ATT diagnostics: 11 writes sent, 12 write handle, 13 char properties, 14 last ATT opcode received,
// 15 last ATT error code, 16 opcode that error answered, 17 ATT notifications received
// link diagnostics: 24 last drop cause (CAUSE_*), 25 reason code of the peer's LL_TERMINATE_IND,
// 26 events skipped (prepared too late), 27 late TX-end ISRs, 28 conn param requests from the peer,
// 29 connection updates applied
// scan diagnostics: 34 SCAN_REQ skipped (ISR too late), 35 CONNECT_IND skipped (ISR too late),
// 36 scan responses naming a cube (ignoring the name filter), 37 ADV_INDs from the target address,
// 38 scan response ISR ran late
static char    last_write[24];
static uint8_t last_write_len;
static char    last_name[3][32];          // [0] from ADV_IND, [1] from SCAN_RSP, [2] the cube we matched
static uint8_t last_name_len[3];

// T_IFS tuning (us, added to "END + 150 us - radio ramp-up"), measured against Circuit Cubes:
// SCAN_REQ was answered at -22..-16 us, CONNECT_IND accepted at 0..-16 us.
static const int SCAN_REQ_ADJ_US = -19;
static const int CONNECT_IND_ADJ_US = -8;

// active scanning
static uint8_t target_addr[6];
static bool    have_target;
static uint8_t cur_adva[6];
static volatile bool rsp_end, rsp_to;

// scanning
static const uint8_t ADV_CH[3] = { 37, 38, 39 };
static uint8_t scan_idx;
static volatile bool scan_rot;
static bool hop_after_rsp;          // a matching scan response arrived: follow the cube to the next channel

// connection (rxbuf is the single TX+RX buffer during a connection, see conn_prepare)
static uint32_t conn_aa, conn_crc;
static uint32_t anchor;             // anchor of the next event to prepare (timer us)
static uint32_t last_rx;            // last valid rx (timer us)
static bool     established;
static uint8_t  chan_unmapped;
static uint16_t conn_event;         // counter of the next event to prepare
static uint16_t cur_event;          // counter of the event prepared last
static uint16_t conn_interval;      // current parameters (BLE units)
static uint16_t conn_latency;
static uint16_t conn_timeout;
static uint8_t  tx_sn, rx_nesn;
static volatile bool rx_end_flag, rx_to, rx_crc_ok;
static bool     version_sent;
static bool     peer_term;          // peer sent LL_TERMINATE_IND
static bool     peer_term_acked;    // ...and an event carrying our ack has been transmitted
static bool     local_term;         // we terminate (not a Circuit Cube) and rescan
static uint8_t  term_cause;

// current LL PDU: kept, and resent unchanged, until the peer acknowledges it. That includes an
// empty PDU: replacing an unacknowledged one with data under the same SN would make the peer
// drop the data as a duplicate while acking it.
static bool     tx_unacked;
static uint8_t  tx_llid, tx_len;
static uint8_t  tx_payload[32];

// queued LL control replies
static const uint8_t CTRL_Q = 3;
static uint8_t  ctrl_q[CTRL_Q][12];
static uint8_t  ctrl_q_len[CTRL_Q];
static uint8_t  ctrl_head, ctrl_count;

// L2CAP signaling (CID 5) reply, and the connection update it may lead to
static uint8_t  sig_rsp[6];
static uint8_t  sig_rsp_len;
static const uint8_t UPD_NONE = 0, UPD_QUEUED = 1, UPD_SENT = 2;
static uint8_t  upd_state;
static uint16_t upd_interval, upd_latency, upd_timeout, upd_instant;

// ATT / GATT
static uint8_t  att_req[8];
static uint8_t  att_req_len;
static bool     att_inflight;
static uint8_t  att_rsp[8];         // reply to a request the peer sent us (we are an ATT server for it)
static uint8_t  att_rsp_len;
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
// TIMER0 has only 4 CC registers (CC[4]/CC[5] exist on TIMER3/4 only). CC0 doubles as the
// "now" scratch register: it is only armed (PPI -> TXEN) between schedule_tx() (<150 us) or
// conn_prepare() (<= PREP_LEAD_US) and the start of the transmission, and tnow() is never
// called inside that window (both ISRs share one priority, the app thread never calls it).
static inline uint32_t tnow() {
    T->TASKS_CAPTURE[0] = 1;
    return T->CC[0];
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
// keep_target: a dropped link rescans for the same cube by address, so it can reconnect on its
// very first ADV_IND instead of first needing a scan response with the name.
static void scan_start(bool keep_target = false) {
    radio_force_disable();
    T->INTENCLR = TIMER_INTENCLR_COMPARE3_Msk;
    g_phase = PH_NONE;
    g_ready = false;
    hop_after_rsp = false;
    if (!keep_target) have_target = false;
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

// Optional extra filter: the advertised name must contain this text (e.g. "4130").
static char name_filter[16];
static volatile uint8_t name_filter_len;

static bool name_contains_filter(const uint8_t *d, uint8_t dl) {
    uint8_t fl = name_filter_len;
    if (fl == 0) return true;
    for (uint8_t i = 0; i + fl <= dl; i++)
        if (memcmp(d + i, name_filter, fl) == 0) return true;
    return false;
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
                if (dl >= tl && memcmp(d, names[k], tl) == 0 && name_contains_filter(d, dl)) return true;
                // shortened name: advertised text is a prefix of the target (cannot satisfy a filter)
                if (type == 0x08 && dl >= 3 && dl < tl && memcmp(d, names[k], dl) == 0 &&
                    name_filter_len == 0) return true;
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
    ll[10] = CONN_INTERVAL_UNITS & 0xFF; ll[11] = CONN_INTERVAL_UNITS >> 8;
    ll[12] = 0; ll[13] = 0;                                 // latency
    ll[14] = CONN_TIMEOUT_UNITS & 0xFF; ll[15] = CONN_TIMEOUT_UNITS >> 8;
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

// After a TX with DISABLED->RXEN armed: wait until the radio really entered RX, so that
// dropping that short afterwards cannot race with it.
static void wait_rx_started() {
    for (volatile int i = 0; i < 300; i++) {
        uint32_t st = R->STATE & 0xF;
        if (st >= 1 && st <= 3) return;     // RXRU / RXIDLE / RX
    }
}

// Transmit txdma T_IFS (+adj us) after the packet that ended at tend (timer us).
// Returns false if too late.
static bool schedule_tx(uint32_t tend, uint32_t shorts, bool then_rx, int adj) {
    uint32_t at = (uint32_t)((int32_t)tend + 150 - (int32_t)TX_RAMP_US + adj);
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
        bool by_name = name_matches(&rxbuf[8], len - 6);
        if (by_name) remember_name(&rxbuf[8], len - 6, 2);
        bool by_addr = have_target && memcmp(adva, target_addr, 6) == 0;
        if (by_addr) stat[37]++;
        bool match = by_name || by_addr;
        if (match && type == 0) {
            stat[5]++;
            build_connect_ind(rxbuf);
            if (schedule_tx(tend, RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk, false, CONNECT_IND_ADJ_US)) {
                stat[6]++;
                T->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
                g_state = ST_INIT_TX;
                return;
            }
            stat[35]++;
        } else if (!match) {
            // name may be in the scan response: ask for it (active scan, like a phone does)
            build_scan_req(rxbuf);
            if (schedule_tx(tend, RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk, true,
                            SCAN_REQ_ADJ_US)) {
                stat[10]++;
                g_state = ST_SCAN_REQ;
                return;
            }
            stat[34]++;
        }
    }
    R->TASKS_START = 1;     // radio is in RXIDLE: keep listening
}

// The scan response is received into txdma, the buffer the SCAN_REQ was sent from: the radio
// latches PACKETPTR ~45 us after our END, so switching buffers here would race with a late ISR
// (and the response would land in the wrong buffer and be lost).
static void scan_req_tx_end() {
    uint32_t tend = T->CC[2];
    tx_trigger_off();
    wait_rx_started();
    R->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;   // no more RXEN
    // the response's ADDRESS event comes ~190 us after our END: only clear our own one
    uint32_t now = tnow();
    if ((int32_t)(now - tend) < 140) clr(&R->EVENTS_ADDRESS);
    else stat[38]++;
    rsp_end = false;
    rsp_to = false;
    uint32_t to = tend + 400;
    if ((int32_t)(to - now) < 5) to = now + 5;      // never arm a compare in the past
    T->CC[3] = to;
    clr(&T->EVENTS_COMPARE[3]);
    T->INTENSET = TIMER_INTENSET_COMPARE3_Msk;
    g_state = ST_SCAN_RSP;
}

static bool name_is_cube(const uint8_t *ad, uint8_t n) {
    const uint8_t *nm;
    uint8_t nl;
    if (!find_name(ad, n, &nm, &nl)) return false;
    return (nl >= 5 && memcmp(nm, "Tenka", 5) == 0) || (nl >= 12 && memcmp(nm, "Circuit Cube", 12) == 0);
}

static void scan_rsp_end() {
    rsp_end = true;
    const uint8_t *b = txdma;
    if (!(R->CRCSTATUS & 1)) return;
    uint8_t len = b[1] & 0x3F;
    if ((b[0] & 0x0F) != 4 || len < 6 || len > 37) return;
    if (memcmp(&b[2], cur_adva, 6) != 0) return;
    stat[4]++;
    remember_name(&b[8], len - 6, 1);
    if (name_is_cube(&b[8], len - 6)) stat[36]++;
    if (name_matches(&b[8], len - 6)) {
        remember_name(&b[8], len - 6, 2);
        memcpy(target_addr, cur_adva, 6);
        have_target = true;     // connect on its next ADV_IND
        // The cube sends the same advertising event on the next channel within a few ms (37 ->
        // 38 -> 39): follow it there instead of waiting a whole advertising interval.
        hop_after_rsp = scan_idx < 2;
    }
}

static void resume_scan() {
    T->INTENCLR = TIMER_INTENCLR_COMPARE3_Msk;
    g_state = ST_SCAN;
    if (hop_after_rsp) {
        hop_after_rsp = false;
        scan_idx++;
        set_channel(ADV_CH[scan_idx]);
        T->CC[1] = tnow() + SCAN_DWELL_US;
    }
    R->SHORTS = RADIO_SHORTS_READY_START_Msk;
    R->PACKETPTR = (uint32_t)rxbuf;
    R->TASKS_RXEN = 1;
}

// ---------------------------------------------------------------------------
// Connection (master side)
// ---------------------------------------------------------------------------
// user-requested disconnect (stays off until connect() is called again)
static volatile bool g_off_req;
static volatile bool g_reconnect;       // connect() called while a disconnect was still in progress
static bool term_sent;

static void enter_idle() {
    radio_force_disable();
    T->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk | TIMER_INTENCLR_COMPARE3_Msk;
    g_phase = PH_NONE;
    g_ready = false;
    g_state = ST_OFF;
    if (g_reconnect) {
        g_reconnect = false;
        g_off_req = false;
        scan_start();
    }
}

// The link is gone: rescan (auto-reconnect) unless the user asked to stay off.
static void conn_leave(uint8_t cause) {
    stat[24] = cause;
    if (cause != CAUSE_USER) stat[9]++;
    if (g_off_req) enter_idle();
    else scan_start(cause != CAUSE_NOT_CUBE);
}

static inline uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }

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
    chan_unmapped = 0;
    conn_event = 0;
    conn_interval = CONN_INTERVAL_UNITS;
    conn_latency = 0;
    conn_timeout = CONN_TIMEOUT_UNITS;
    tx_sn = rx_nesn = 0;
    tx_unacked = false;
    ctrl_head = ctrl_count = 0;
    sig_rsp_len = 0;
    upd_state = UPD_NONE;
    version_sent = false;
    term_sent = false;
    peer_term = peer_term_acked = false;
    local_term = false;
    att_rsp_len = 0;
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

static void wrap_l2cap(uint8_t cid, const uint8_t *p, uint8_t n) {
    tx_payload[0] = n; tx_payload[1] = 0; tx_payload[2] = cid; tx_payload[3] = 0;
    memcpy(tx_payload + 4, p, n);
    tx_len = n + 4;
    tx_llid = 2;
}
static inline void wrap_att(const uint8_t *att, uint8_t n) { wrap_l2cap(4, att, n); }

static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }

// Pick the next PDU to send; leaves an empty PDU (LLID 1, length 0) when nothing is queued.
static void fetch_tx() {
    tx_llid = 1;
    tx_len = 0;
    if (ctrl_count) {
        tx_len = ctrl_q_len[ctrl_head];
        memcpy(tx_payload, ctrl_q[ctrl_head], tx_len);
        tx_llid = 3;
        ctrl_head = (ctrl_head + 1) % CTRL_Q;
        ctrl_count--;
        return;
    }
    if (sig_rsp_len) {
        wrap_l2cap(5, sig_rsp, sig_rsp_len);
        sig_rsp_len = 0;
        return;
    }
    if (upd_state == UPD_QUEUED) {
        // LL_CONNECTION_UPDATE_IND. With WinOffset 0 the new timing starts at the instant's
        // anchor (old interval), so the master simply switches interval at the instant.
        upd_instant = cur_event + UPD_INSTANT_EVENTS;
        uint8_t *c = tx_payload;
        c[0] = 0x00;
        c[1] = 1;                       // WinSize 1.25 ms
        put16(c + 2, 0);                // WinOffset
        put16(c + 4, upd_interval);
        put16(c + 6, upd_latency);
        put16(c + 8, upd_timeout);
        put16(c + 10, upd_instant);
        tx_len = 12;
        tx_llid = 3;
        upd_state = UPD_SENT;
        return;
    }
    if (att_rsp_len) {
        wrap_att(att_rsp, att_rsp_len);
        att_rsp_len = 0;
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
        stat[11]++;
        last_write_len = n - 3;
        memcpy(last_write, att + 3, last_write_len);
        wrap_att(att, n);
        if (!no_rsp) {
            att_wait_rsp = true;
            att_wait_until = tnow() + 1000000;
        }
    }
}

static void ctrl_reply(const uint8_t *p, uint8_t n) {
    if (ctrl_count >= CTRL_Q || n > sizeof(ctrl_q[0])) return;
    uint8_t i = (ctrl_head + ctrl_count) % CTRL_Q;
    memcpy(ctrl_q[i], p, n);
    ctrl_q_len[i] = n;
    ctrl_count++;
}

static void handle_ll_control(const uint8_t *d, uint8_t len) {
    if (len < 1) return;
    uint8_t op = d[0];
    switch (op) {
    case 0x02:  // LL_TERMINATE_IND: ack it in the next event, then leave
        peer_term = true;
        if (len >= 2) stat[25] = d[1];
        break;
    case 0x14: {// LL_LENGTH_REQ: keep the default 27 bytes / 328 us both ways
        uint8_t r[9] = { 0x15, 27, 0, 0x48, 0x01, 27, 0, 0x48, 0x01 };
        ctrl_reply(r, 9);
        break; }
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
    case 0x07: case 0x09: case 0x0D: case 0x11: case 0x13: case 0x15: case 0x19:
        break;  // responses / indications needing no reply: ignore
    default: {
        uint8_t r[2] = { 0x07, op };            // LL_UNKNOWN_RSP
        ctrl_reply(r, 2);
        break; }
    }
}

static void att_rx(const uint8_t *p, uint8_t n) {
    if (n < 1) return;
    stat[14] = p[0];
    if (p[0] == 0x1B) stat[17]++;
    if (p[0] == 0x01 && n >= 5) { stat[16] = p[1]; stat[15] = p[4]; }
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
                stat[12] = gatt_handle;
                stat[13] = gatt_props;
                g_ready = true;
                return;
            }
        }
        if (last && last < 0xFFFF) att_request_discovery(last + 1);
        else local_term = true;         // reached end: not a Circuit Cube
        break; }
    case 0x01:      // Error Response
        if (n >= 2 && p[1] == 0x08) {
            att_inflight = false;
            if (!g_ready) local_term = true;        // Attribute Not Found: no NUS write char
        } else if (n >= 2 && (p[1] == 0x12 || p[1] == 0x52)) {
            att_wait_rsp = false;
        }
        break;
    case 0x13:      // Write Response
        att_wait_rsp = false;
        break;
    case 0x02: {    // Exchange MTU Request from the peer: stay at the default MTU of 23
        static const uint8_t r[3] = { 0x03, 23, 0 };
        memcpy(att_rsp, r, 3);
        att_rsp_len = 3;
        break; }
    case 0x1D: {    // Handle Value Indication: confirm
        att_rsp[0] = 0x1E;
        att_rsp_len = 1;
        break; }
    // Requests the peer sends us. An unanswered request stalls its ATT bearer and after 30 s
    // most stacks drop the link, so every request gets an Error Response.
    case 0x04: case 0x06: case 0x08: case 0x10:     // discovery: our database is empty
        att_rsp[0] = 0x01;
        att_rsp[1] = p[0];
        att_rsp[2] = n >= 3 ? p[1] : 0; att_rsp[3] = n >= 3 ? p[2] : 0;
        att_rsp[4] = 0x0A;          // Attribute Not Found
        att_rsp_len = 5;
        break;
    case 0x0A: case 0x0C: case 0x0E: case 0x12: case 0x16: case 0x18: case 0x20:
        att_rsp[0] = 0x01;
        att_rsp[1] = p[0];
        att_rsp[2] = 0; att_rsp[3] = 0;
        att_rsp[4] = 0x06;          // Request Not Supported
        att_rsp_len = 5;
        break;
    default:
        break;      // notifications, etc.: ignored
    }
}

// L2CAP LE signaling channel. Peripherals ask for their preferred connection parameters with a
// Connection Parameter Update Request; many (e.g. Nordic's ble_conn_params) disconnect after a
// few unanswered or failed attempts, so accept it and apply it with LL_CONNECTION_UPDATE_IND.
static void sig_rx(const uint8_t *p, uint16_t n) {
    if (n < 4) return;
    uint8_t code = p[0], id = p[1];
    if (code == 0x12 && n >= 12) {          // Connection Parameter Update Request
        stat[28]++;
        uint16_t imin = rd16(p + 4), imax = rd16(p + 6), lat = rd16(p + 8), to = rd16(p + 10);
        uint16_t iv = CONN_INTERVAL_UNITS;
        if (iv < imin) iv = imin;
        if (iv > imax) iv = imax;
        // spec limits; supervision timeout must exceed (1 + latency) * interval * 2
        bool ok = imin >= 6 && imin <= imax && imax <= 3200 && lat <= 499 &&
                  to >= 10 && to <= 3200 && (uint32_t)to * 4 > (uint32_t)(1 + lat) * iv &&
                  upd_state == UPD_NONE;
        sig_rsp[0] = 0x13; sig_rsp[1] = id; sig_rsp[2] = 2; sig_rsp[3] = 0;
        sig_rsp[4] = ok ? 0 : 1; sig_rsp[5] = 0;  // 0 accepted, 1 rejected
        sig_rsp_len = 6;
        if (ok && (iv != conn_interval || lat != conn_latency || to != conn_timeout)) {
            upd_interval = iv;
            upd_latency = lat;
            upd_timeout = to;
            upd_state = UPD_QUEUED;     // sent by fetch_tx after the response above
        }
    } else if (code == 0x06 || code == 0x14 || code == 0x17 || code == 0x19) {
        // other requests: Command Reject, "command not understood"
        sig_rsp[0] = 0x01; sig_rsp[1] = id; sig_rsp[2] = 2; sig_rsp[3] = 0;
        sig_rsp[4] = 0; sig_rsp[5] = 0;
        sig_rsp_len = 6;
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
        tx_unacked = false;
    }
    if (sn == rx_nesn) {            // new data from peer
        rx_nesn ^= 1;
        const uint8_t *d = &rxbuf[2];
        if (llid == 3) {
            handle_ll_control(d, len);
        } else if (llid == 2 && len >= 4) {     // L2CAP start (MTU 23: never fragmented)
            uint16_t l2len = rd16(d);
            uint16_t cid = rd16(d + 2);
            if (l2len + 4 > len) return;
            if (cid == 4) att_rx(d + 4, l2len);
            else if (cid == 5) sig_rx(d + 4, l2len);
        }
    }
}

static void finish_event(bool got) {
    g_phase = PH_NONE;
    T->INTENCLR = TIMER_INTENCLR_COMPARE3_Msk;
    if (got) {
        stat[7]++;
        process_rx();
        last_rx = tnow();
        established = true;
    } else {
        stat[8]++;
    }
    if (term_sent && !tx_unacked) conn_leave(term_cause);   // our terminate acknowledged
    else if (peer_term && peer_term_acked) conn_leave(CAUSE_PEER);
    // supervision timeout is checked in conn_prepare, which also runs when an event is lost
}

// Advance to the next connection event: counter, channel, anchor. Called once per event even
// when the event is skipped, so hopping and the update instant stay in step with the peer.
static uint32_t next_event() {
    uint32_t a = anchor;
    cur_event = conn_event++;
    chan_unmapped = (chan_unmapped + CONN_HOP) % 37;
    if (upd_state == UPD_SENT && cur_event == upd_instant) {
        conn_interval = upd_interval;
        conn_latency = upd_latency;
        conn_timeout = upd_timeout;
        upd_state = UPD_NONE;
        stat[29]++;
    }
    anchor += (uint32_t)conn_interval * 1250;
    return a;
}

static void conn_prepare() {
    g_phase = PH_NONE;
    T->INTENCLR = TIMER_INTENCLR_COMPARE3_Msk;
    if (R->STATE & 0xF) radio_force_disable();
    uint32_t now = tnow();

    uint32_t limit = established ? (uint32_t)conn_timeout * 10000 : CONN_ESTABLISH_US;
    if ((int32_t)(now - last_rx) > (int32_t)limit) {
        conn_leave(established ? CAUSE_TIMEOUT : CAUSE_ESTABLISH);
        return;
    }

    // Skip (but still count) every event whose TX time is already too close or past.
    uint32_t at;
    for (;;) {
        at = next_event() - TX_RAMP_US;
        if ((int32_t)(at - now) > (int32_t)PREP_MIN_US) break;
        stat[26]++;
    }
    T->CC[1] = anchor - PREP_LEAD_US;
    set_channel(chan_unmapped);

    if (!tx_unacked) {              // previous PDU acknowledged: pick a new one
        if ((g_off_req || local_term) && !term_sent) {
            // LL_TERMINATE_IND, 0x13 remote user terminated
            tx_payload[0] = 0x02;
            tx_payload[1] = 0x13;
            tx_len = 2;
            tx_llid = 3;
            term_sent = true;
            term_cause = g_off_req ? CAUSE_USER : CAUSE_NOT_CUBE;
        } else if (peer_term || term_sent) {
            tx_llid = 1;
            tx_len = 0;
        } else {
            fetch_tx();
        }
    }
    if (peer_term) peer_term_acked = true;      // this PDU carries the ack (NESN)

    // One buffer for TX and the reply: the reply is received into the same memory right after
    // our packet, so a late ISR cannot leave PACKETPTR pointing at the wrong buffer.
    rxbuf[0] = tx_llid | (rx_nesn << 2) | (tx_sn << 3);
    rxbuf[1] = tx_len;
    memcpy(&rxbuf[2], tx_payload, tx_len);
    tx_unacked = true;

    rx_end_flag = false;
    rx_to = false;
    clr(&R->EVENTS_END);
    clr(&R->EVENTS_DISABLED);
    clr(&R->EVENTS_ADDRESS);
    R->PACKETPTR = (uint32_t)rxbuf;
    R->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |
                RADIO_SHORTS_DISABLED_RXEN_Msk;
    g_phase = PH_TX;
    T->CC[0] = at;
    clr(&T->EVENTS_COMPARE[0]);
    tx_trigger_on();
}

static void conn_tx_end() {
    uint32_t tend = T->CC[2];
    tx_trigger_off();
    // The reply's preamble starts 150 us after our END, its ADDRESS event ~40 us later. Clear our
    // own TX ADDRESS event only while that cannot have happened yet; when this ISR runs late, keep
    // it (no rx timeout this event, conn_prepare cleans up) rather than cut off a reply mid-air.
    if ((int32_t)(tnow() - tend) < 140) clr(&R->EVENTS_ADDRESS);
    else stat[27]++;
    wait_rx_started();
    R->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;   // no more RXEN
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
            T->CC[1] += SCAN_DWELL_US;      // no tnow(): CC0 may be armed for a TX right now
            if (g_state == ST_SCAN) {
                if (g_off_req) {
                    enter_idle();
                } else {
                    scan_rot = true;
                    R->TASKS_DISABLE = 1;
                }
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
    // Both at the highest priority: CODAL's serial/GPIOTE ISRs run at 1-2 and must not delay
    // connection events. Equal priority also means these two never preempt each other.
    NVIC_SetPriority(RADIO_IRQn, 0);
    NVIC_SetPriority(CC_TIMER_IRQn, 0);
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_ClearPendingIRQ(CC_TIMER_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(CC_TIMER_IRQn);
    g_inited = true;
}

// API call counters (debugStat 32/33): tell a user disconnect apart from a link drop
static uint32_t n_connect_calls, n_disconnect_calls;

//%
void startConnect() {
    n_connect_calls++;
    if (!g_inited) stack_init();
    if (g_off_req && g_state != ST_OFF) {
        g_reconnect = true;         // restart as soon as the disconnect has finished
        return;
    }
    g_off_req = false;
    if (g_state == ST_OFF) scan_start();
}

//%
void setNameFilter(String name) {
    name_filter_len = 0;
    if (!name) return;
    int n = name->getUTF8Size();
    if (n > 15) n = 15;
    memcpy(name_filter, name->getUTF8Data(), n);
    name_filter_len = n;
}

//%
void disconnect() {
    n_disconnect_calls++;
    if (!g_inited || g_state == ST_OFF) return;
    g_reconnect = false;
    g_off_req = true;
}

//%
bool isConnected() {
    return g_inited && g_state == ST_CONN && g_ready && !g_off_req;
}

//%
void setMotorRaw(int port, int speed) {
    stat[18]++;
    if (port < 1 || port > 3 || !isConnected()) return;
    stat[19]++;
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    g_speed[port - 1] = speed;
    __atomic_fetch_or(&g_dirty, 1 << (port - 1), __ATOMIC_ACQ_REL);
}

//%
int debugStat(int i) {
    if (i == 0) return g_state;
    // live queue state: 20 dirty mask, 21 LL PDU waiting for ack, 22 ATT request in flight, 23 ATT reply queued
    if (i == 20) return g_dirty;
    if (i == 21) return tx_unacked;
    if (i == 22) return att_inflight;
    if (i == 23) return att_rsp_len;
    // 30 current connection interval (ms), 31 current supervision timeout (ms)
    if (i == 30) return conn_interval * 5 / 4;
    if (i == 31) return conn_timeout * 10;
    // 32 connect() calls, 33 disconnect() calls
    if (i == 32) return n_connect_calls;
    if (i == 33) return n_disconnect_calls;
    if (i < 1 || i > 39) return 0;
    return (int)stat[i];
}

//%
String debugName(int kind) {
    if (kind == 3) return mkString(last_write, last_write_len);   // last motor command written
    if (kind < 0 || kind > 2) kind = 0;
    return mkString(last_name[kind], last_name_len[kind]);
}

//%
void stopAll() {
    if (!isConnected()) return;
    for (int i = 0; i < 3; i++) g_speed[i] = 0;
    __atomic_fetch_or(&g_dirty, 7, __ATOMIC_ACQ_REL);
}

} // namespace circuitcubes

// Lets the TypeScript layer tell hardware from the simulator (which returns "sim").
namespace control {
//%
String deviceDalVersion() {
    return mkString("hw", 2);
}
}

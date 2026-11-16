// Auto-generated. Do not edit.


    /**
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
     *   - NRF_TIMER4 (free running, 1 MHz) + TIMER4_IRQn
     *   - PPI channels 18 and 19
     *
     * Do NOT combine with the MakeCode "bluetooth" or "radio" extensions.
     */

declare namespace circuitcubes {
}

// Auto-generated. Do not edit. Really.

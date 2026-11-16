# pxt-circuitcubes

MakeCode extension that lets a **micro:bit v2** control a **Circuit Cubes Bluetooth
Battery Cube** directly: the micro:bit is the BLE *Central*, no phone or PC needed.

> **Status: compiles and links for micro:bit v2 (CODAL); not yet tested on real hardware.**
> The radio timing code (T_IFS, connection events) is written from the spec and
> needs bring-up against a real cube. See *Known limitations*.

## Blocks

```blocks
circuitCubes.connect()
circuitCubes.isConnected()
circuitCubes.setMotor(CubePort.A, 255)   // -255..255, 0 = stop
circuitCubes.stopAll()
```

| Block | What it does |
| --- | --- |
| `connect to Circuit Cube` | Starts scanning for a device named `Tenka` or `Circuit Cube`, connects, discovers the Nordic UART write characteristic. Reconnects automatically after a dropped link. |
| `Circuit Cube is connected` | `true` once linked **and** the write characteristic was found. |
| `set motor A/B/C to speed n` | Clamps to -255..255, sends `[+/-]NNN[a/b/c]`. Ignored while not connected. |
| `stop all motors` | Sets all three ports to 0. |

Typical program:

```ts
circuitCubes.connect()
basic.forever(function () {
    if (circuitCubes.isConnected()) {
        circuitCubes.setMotor(CubePort.A, 200)
    }
})
```

## How it works

* `pxt.json` depends only on `core`, targets micro:bit and disables the v1 (`mbdal`) variant.
  Do **not** add the `bluetooth` or `radio` extensions: they own `NRF_RADIO`.
* `circuitcubes.cpp` disables the SoftDevice if one is enabled (`sd_softdevice_disable`),
  starts HFXO, and drives `NRF_RADIO` directly (BLE 1 Mbit, whitening, CRC24):
  * scan 37/38/39, match the local name in `ADV_IND`, answer with `CONNECT_IND` 150 µs after it
  * master connection events (30 ms interval, 2 s supervision timeout, all 37 channels, CSA#1),
    SN/NESN handling, replies to LL feature/version/ping/terminate, `LL_UNKNOWN_RSP` otherwise
  * ATT *Read By Type* (`0x2803`) to find characteristic `6e400002-b5a3-f393-e0a9-e50e24dcca9e`,
    then ATT *Write Command* `0x52` (or *Write Request* `0x12` if the characteristic only supports that)
  * latest-value-wins per port: only the newest speed of each port is sent, up to three commands per packet
* Resources used (all verified unused by CODAL v0.3.5 of `codal-microbit-v2`):
  `TIMER0` (SoftDevice's timer, free once the SD is off), PPI channels 18/19.
  CODAL already defines `RADIO_IRQHandler` for its own radio driver, so the ISRs are installed with
  `NVIC_SetVector` instead of overriding that symbol.

## Known limitations

* Untested on hardware. Likeliest tuning points are the constants `TX_RAMP_US` (CONNECT_IND and
  anchor timing) and `PREP_LEAD_US` at the top of `circuitcubes.cpp`.
* The name must be in the `ADV_IND`; a name only present in a scan response is not matched.
* No encryption/pairing, no MTU exchange (default 23), no notifications (RX characteristic unused),
  one packet pair per connection event.
* Zero speed is sent as `+000x` (5 bytes). Set `CC_SIGNED_STOP 0` for the 4-byte `000x` form.
* The simulator only fakes connection state.
* Using this extension takes the radio away from CODAL; the SoftDevice stays off until reset.

## Building

```sh
npm i -g pxt
pxt target microbit && pxt install   # once
./tools/build.sh                     # -> built/mbcodal-binary.hex (micro:bit v2 only)
```

Copy `built/mbcodal-binary.hex` to the `MICROBIT` drive. The test program in `test.ts`
is included: the matrix shows a cross until the cube connects, then a tick; button B toggles
motor A, A+B stops all.

`pxt build` runs CODAL in the Docker image `pext/yotta:latest`, which current Docker can no
longer pull, so `tools/shim/docker` redirects that one call to the host toolchain
(`arm-none-eabi-gcc`, `cmake`, `python3`; it sets `CMAKE_POLICY_VERSION_MINIMUM=3.5` for CMake 4).
The legacy v1 stage still runs once in Docker (its image still pulls) but its output is not used.

## License

MIT

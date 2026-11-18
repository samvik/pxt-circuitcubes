# pxt-circuitcubes

MakeCode extension that lets a **micro:bit v2** control a **Circuit Cubes Bluetooth
Battery Cube** directly: the micro:bit is the BLE *Central*, no phone or PC needed.

> **Status:** works with one Circuit Cube (`Tenka…`) on one micro:bit v2: connects, stays connected
> (~1 % of connection events missed) and drives motors. Other cubes/boards are untested, and only one
> cube can be connected at a time.

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
| `wait until Circuit Cube is connected (max N ms)` | Pauses the program until connected; `0` (default) waits forever. |
| `on Circuit Cube connected` / `on Circuit Cube disconnected` | Events fired whenever the link comes up or goes down (including automatic reconnects and drops). One handler each; polled every 50 ms. |
| `disconnect Circuit Cube` | Closes the link (LL terminate) and stops scanning/auto-reconnect until `connect` is called again. |
| `set motor A/B/C to speed n` | Clamps to -255..255, sends `[+/-]NNN[a/b/c]`. Ignored while not connected. |
| `run motor A/B/C at speed n for N ms` | Runs the motor, waits, then stops it. |
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

## Robot blocks

For a tank-style robot (one motor per track) there is a **Robot** sub-tab under *Circuit Cubes*
(`circuitCubes.robot`, in `robot.ts`). Speeds are **percent**, -100..100, converted to the cube's
-255..255. Steering works by varying the speed of the two tracks.

```blocks
circuitCubes.robot.setMotors(CubePort.A, CubePort.B)
circuitCubes.robot.steer(30, 50)
circuitCubes.robot.stop()
```

| Block | What it does |
| --- | --- |
| `robot left motor A right motor B` | Chooses which cube ports drive the left and right track (default A and B). Port C stays free for other uses. |
| `robot reverse left/right motor on/off` | Reverses one track. Use it when a motor is mounted mirrored and the robot spins instead of driving straight. |
| `robot drive left L% right R%` | Sets each track's speed independently. |
| `robot drive forward/backward at S%` | Both tracks at the same speed. |
| `robot steer D at S%` | Drives in an arc. `D` is -100 (left) to 100 (right): 0 is straight, ±100 stops the inner track. A negative speed reverses. The inner track runs at `S × (100 − |D|) / 100`. |
| `robot turn left/right at S%` | Pivots on the spot: one track forward, the other backward. |
| `robot stop` | Stops both tracks. |
| `robot drive … for N ms`, `robot turn … for N ms`, `robot steer … for N ms` | Timed versions; they wait, then stop the robot. |
| `robot left speed` / `robot right speed` | The current speed of each track in percent (the logical value, before any reversal). |

Example: steer with the micro:bit's tilt (forward/back = speed, left/right = steering). If a
direction is the wrong way round, flip the sign of the matching `input.rotation` term.

```ts
circuitCubes.connect()
circuitCubes.waitUntilConnected()
circuitCubes.robot.setMotors(CubePort.A, CubePort.B)

const DEADZONE = 10   // degrees ignored, so a flat board stands still
const MAX_TILT = 40   // degrees giving full speed / full steering

function tiltToPercent(angle: number): number {
    if (Math.abs(angle) < DEADZONE) return 0
    const sign = angle < 0 ? -1 : 1
    return sign * Math.min(100, (Math.abs(angle) - DEADZONE) * 100 / (MAX_TILT - DEADZONE))
}

basic.forever(function () {
    if (circuitCubes.isConnected()) {
        const speed = tiltToPercent(-input.rotation(Rotation.Pitch))
        const steer = tiltToPercent(input.rotation(Rotation.Roll))
        if (speed == 0 && steer != 0) {
            circuitCubes.robot.turn(steer < 0 ? TurnDirection.Left : TurnDirection.Right, Math.abs(steer))
        } else {
            circuitCubes.robot.steer(steer, speed)
        }
    }
    basic.pause(50)   // each update sends two BLE writes; raise this if the link drops
})
```

## How it works

* `pxt.json` depends only on `core`, targets micro:bit and disables the v1 (`mbdal`) variant.
  Do **not** add the `bluetooth` or `radio` extensions: they own `NRF_RADIO`.
* `circuitcubes.cpp` disables the SoftDevice if one is enabled (`sd_softdevice_disable`),
  starts HFXO, and drives `NRF_RADIO` directly (BLE 1 Mbit, whitening, CRC24):
  * scan 37/38/39, match the local name in `ADV_IND`, answer with `CONNECT_IND` 150 µs after it
  * master connection events (30 ms interval, 4 s supervision timeout, all 37 channels, CSA#1),
    SN/NESN handling (an unacknowledged PDU, empty or not, is resent unchanged), replies to
    LL feature/version/ping/length/terminate, `LL_UNKNOWN_RSP` otherwise
  * events the CPU was too late for are skipped but still counted, so channel hopping stays in
    step with the cube
  * L2CAP *Connection Parameter Update Requests* from the cube are accepted and applied with
    `LL_CONNECTION_UPDATE_IND` (interval closest to 30 ms in the requested range)
  * ATT *Read By Type* (`0x2803`) to find characteristic `6e400002-b5a3-f393-e0a9-e50e24dcca9e`,
    then ATT *Write Command* `0x52` (or *Write Request* `0x12` if the characteristic only supports that)
  * latest-value-wins per port: only the newest speed of each port is sent, up to three commands per packet
* Resources used (all verified unused by CODAL v0.3.5 of `codal-microbit-v2`):
  `TIMER0` (SoftDevice's timer, free once the SD is off), PPI channels 18/19.
  CODAL already defines `RADIO_IRQHandler` for its own radio driver, so the ISRs are installed with
  `NVIC_SetVector` instead of overriding that symbol.

## Incompatible extensions

This extension owns the radio, so it must not be combined with `radio` or `bluetooth`.
MakeCode has no "conflicts" field in `pxt.json`; the only conflict check it performs is on
build settings. `radio` sets `microbit-dal.bluetooth.enabled = 0` and `bluetooth` sets it to `1`,
so `pxt.json` here declares the otherwise unused value `2`. Adding this extension to a project that
has `radio`/`bluetooth` (or the reverse) then triggers MakeCode's "extension conflict" dialog, which
offers to remove the conflicting extension. The setting itself has no effect on the build.
Other extensions that use the radio but set no such value cannot be detected this way.

## Known limitations

* One cube at a time. Timing constants to tune if another cube/board misbehaves: `TX_RAMP_US`,
  `PREP_LEAD_US`, `SCAN_REQ_ADJ_US` and `CONNECT_IND_ADJ_US` at the top of `circuitcubes.cpp`.
* If the link still drops, the test program's `link:` serial line tells why: `cause` 1 = supervision
  timeout (radio/timing), 3 = the cube terminated (`peerReason` is its HCI reason code, e.g.
  `0x3B`=59 unacceptable connection parameters), and `skipped`/`lateIsr` count events the
  CPU was too busy for.
* Cubes put their name in the scan response, so the scanner is *active* (it sends SCAN_REQ to every
  connectable advertiser nearby).
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

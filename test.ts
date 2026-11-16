// Test harness. Matrix: X while not connected, tick when connected.
// B toggles motor A, A+B stops all. Radio diagnostics go to serial (115200 baud)
// once per second; hold A to also scroll the counters on the LED matrix.
circuitCubes.connect()
let running = false
input.onButtonPressed(Button.B, function () {
    running = !running
    circuitCubes.setMotor(CubePort.A, running ? 255 : 0)
})
input.onButtonPressed(Button.AB, function () {
    circuitCubes.stopAll()
})
basic.forever(function () {
    if (circuitCubes.isConnected()) {
        basic.showIcon(IconNames.Yes)
    } else {
        basic.showIcon(IconNames.No)
    }
})
control.inBackground(function () {
    while (true) {
        serial.writeLine("st=" + circuitCubes.debugStat(0)
            + " adv=" + circuitCubes.debugStat(1)
            + " bad=" + circuitCubes.debugStat(2)
            + " ind=" + circuitCubes.debugStat(3)
            + " sreq=" + circuitCubes.debugStat(10)
            + " rsp=" + circuitCubes.debugStat(4)
            + " match=" + circuitCubes.debugStat(5)
            + " conn=" + circuitCubes.debugStat(6)
            + " ok=" + circuitCubes.debugStat(7)
            + " miss=" + circuitCubes.debugStat(8)
            + " drop=" + circuitCubes.debugStat(9)
            + " nameAdv='" + circuitCubes.debugName(0)
            + "' nameRsp='" + circuitCubes.debugName(1) + "'")
        basic.pause(1000)
    }
})

// Test harness. Connects only to the cube whose name contains "C25E" (change it to your cube). Matrix: X while not connected, tick when connected.
// A: run motor A for 1 s   B: toggle motor A   A+B: stop all
// Logo touch: disconnect / connect again. Radio diagnostics go to serial (115200 baud) once per second.
circuitCubes.connectToNamed("C25E")
let running = false
let wantConnected = true
input.onButtonPressed(Button.A, function () {
    circuitCubes.runMotorFor(CubePort.A, 255, 1000)
})
input.onButtonPressed(Button.B, function () {
    running = !running
    circuitCubes.setMotor(CubePort.A, running ? 255 : 0)
})
input.onButtonPressed(Button.AB, function () {
    circuitCubes.stopAll()
})
input.onLogoEvent(TouchButtonEvent.Pressed, function () {
    wantConnected = !wantConnected
    if (wantConnected) {
        circuitCubes.connectToNamed("C25E")
    } else {
        circuitCubes.disconnect()
    }
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
            + "' nameRsp='" + circuitCubes.debugName(1)
            + "' cube='" + circuitCubes.debugName(2) + "'")
        serial.writeLine("  att: writes=" + circuitCubes.debugStat(11)
            + " handle=" + circuitCubes.debugStat(12)
            + " props=" + circuitCubes.debugStat(13)
            + " lastOp=" + circuitCubes.debugStat(14)
            + " errCode=" + circuitCubes.debugStat(15)
            + " errOp=" + circuitCubes.debugStat(16)
            + " notif=" + circuitCubes.debugStat(17)
            + " setMotorCalls=" + circuitCubes.debugStat(18)
            + " accepted=" + circuitCubes.debugStat(19)
            + " dirty=" + circuitCubes.debugStat(20)
            + " txHas=" + circuitCubes.debugStat(21)
            + " attInflight=" + circuitCubes.debugStat(22)
            + " lastWrite='" + circuitCubes.debugName(3) + "'")
        basic.pause(1000)
    }
})

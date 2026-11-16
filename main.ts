/**
 * Circuit Cubes Bluetooth Battery Cube control for micro:bit v2.
 *
 * The micro:bit acts as a BLE *Central* (scanner + initiator) and talks to the
 * cube over the Nordic UART Service. All radio work is done in C++
 * (circuitcubes.cpp); this file only exposes the blocks.
 */

enum CubePort {
    //% block="A"
    A = 1,
    //% block="B"
    B = 2,
    //% block="C"
    C = 3
}

//% weight=90 color=#E8A200 icon="" block="Circuit Cubes"
namespace circuitCubes {

    // ---- simulator state (the simulator has no radio) ----------------------
    let simConnected = false;

    // ---- native shims -------------------------------------------------------
    // On hardware these bind to circuitcubes.cpp. The bodies are only used by
    // the simulator.

    //% shim=circuitcubes::startConnect
    function nativeConnect(): void {
        simConnected = true;
    }

    //% shim=circuitcubes::isConnected
    function nativeIsConnected(): boolean {
        return simConnected;
    }

    //% shim=circuitcubes::setMotorRaw
    function nativeSetMotor(port: number, speed: number): void {
        // simulator: nothing to drive
    }

    //% shim=circuitcubes::stopAll
    function nativeStopAll(): void {
        // simulator: nothing to drive
    }

    // ---- diagnostics (hidden) ------------------------------------------------

    /**
     * Radio counters: 0 state(1 scan,3 connected), 1 adv ok, 2 adv bad CRC, 3 ADV_IND,
     * 4 SCAN_RSP, 5 name match, 6 CONNECT_IND sent, 7 events answered, 8 events missed, 9 disconnects
     */
    //% shim=circuitcubes::debugStat blockHidden=true
    export function debugStat(index: number): number {
        return 0;
    }

    /** Last device name heard: kind 0 = advertisement, 1 = scan response */
    //% shim=circuitcubes::debugName blockHidden=true
    export function debugName(kind: number): string {
        return "";
    }

    // ---- blocks -------------------------------------------------------------

    /**
     * Start scanning for a Circuit Cube ("Tenka" / "Circuit Cube") and connect
     * to the first one found. Returns immediately; use "cube is connected"
     * to wait for the link. Reconnects automatically if the link drops.
     */
    //% blockId=circuitcubes_connect block="connect to Circuit Cube"
    //% weight=100
    export function connect(): void {
        nativeConnect();
    }

    /**
     * True when connected to a cube and its motor characteristic was found.
     */
    //% blockId=circuitcubes_isconnected block="Circuit Cube is connected"
    //% weight=90
    export function isConnected(): boolean {
        return nativeIsConnected();
    }

    /**
     * Set the speed and direction of a motor port.
     * @param port which motor port, eg: CubePort.A
     * @param speed -255 (full reverse) to 255 (full forward), 0 stops, eg: 255
     */
    //% blockId=circuitcubes_setmotor block="set motor %port to speed %speed"
    //% speed.min=-255 speed.max=255 speed.defl=255
    //% weight=80
    export function setMotor(port: CubePort, speed: number): void {
        if (port < CubePort.A || port > CubePort.C) return;
        speed = Math.round(speed);
        if (speed > 255) speed = 255;
        if (speed < -255) speed = -255;
        nativeSetMotor(port, speed);
    }

    /**
     * Stop all three motor ports.
     */
    //% blockId=circuitcubes_stopall block="stop all motors"
    //% weight=70
    export function stopAll(): void {
        nativeStopAll();
    }
}

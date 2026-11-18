/**
 * Circuit Cubes Bluetooth Battery Cube control for micro:bit v2.
 *
 * The micro:bit acts as a BLE *Central* (scanner + initiator) and talks to the
 * cube over the Nordic UART Service. All radio work is done in C++
 * (circuitcubes.cpp); this file only exposes the blocks.
 */

declare namespace control {
    //% shim=control::deviceDalVersion
    function deviceDalVersion(): string;
}

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
    // On hardware these bind to circuitcubes.cpp. The simulator has no
    // implementation of them, so every call is guarded with isSim().

    // control.deviceDalVersion() exists in the simulator (returns "sim"); on hardware
    // it is provided by circuitcubes.cpp and returns something else.
    function isSim(): boolean {
        return control.deviceDalVersion() == "sim";
    }

    //% shim=circuitcubes::startConnect
    function hwConnect(): void { }

    //% shim=circuitcubes::isConnected
    function hwIsConnected(): boolean { return false; }

    //% shim=circuitcubes::setMotorRaw
    function hwSetMotor(port: number, speed: number): void { }

    //% shim=circuitcubes::setNameFilter
    function hwSetNameFilter(name: string): void { }

    //% shim=circuitcubes::disconnect
    function hwDisconnect(): void { }

    //% shim=circuitcubes::stopAll
    function hwStopAll(): void { }

    function nativeConnect(): void {
        if (isSim()) simConnected = true; else hwConnect();
    }
    function nativeIsConnected(): boolean {
        return isSim() ? simConnected : hwIsConnected();
    }
    function nativeSetMotor(port: number, speed: number): void {
        if (!isSim()) hwSetMotor(port, speed);
    }
    function nativeSetNameFilter(name: string): void {
        if (!isSim()) hwSetNameFilter(name);
    }
    function nativeDisconnect(): void {
        if (isSim()) simConnected = false; else hwDisconnect();
    }
    function nativeStopAll(): void {
        if (!isSim()) hwStopAll();
    }

    // ---- diagnostics (hidden) ------------------------------------------------

    /**
     * Radio counters: 0 state(1 scan,3 connected), 1 adv ok, 2 adv bad CRC, 3 ADV_IND,
     * 4 SCAN_RSP, 5 name match, 6 CONNECT_IND sent, 7 events answered, 8 events missed, 9 disconnects,
     * 24 last drop cause (1 supervision timeout, 2 failed to establish, 3 cube terminated,
     * 4 not a Circuit Cube, 5 user disconnect), 25 cube's terminate reason code,
     * 26 events skipped (prepared too late), 27 late TX-end interrupts,
     * 28 connection parameter requests from the cube, 29 connection updates applied,
     * 30 connection interval (ms), 31 supervision timeout (ms), 32 connect() calls, 33 disconnect() calls,
     * 34 SCAN_REQ skipped (too late), 35 CONNECT_IND skipped (too late), 36 scan responses naming
     * any cube, 37 adverts from the target address, 38 scan-response interrupt ran late
     */
    //% blockHidden=true
    export function debugStat(index: number): number {
        return isSim() ? 0 : hwDebugStat(index);
    }

    //% shim=circuitcubes::debugStat
    function hwDebugStat(index: number): number { return 0; }

    /** Names: kind 0 = last advertisement, 1 = last scan response, 2 = the cube that was matched, 3 = last motor command written */
    //% blockHidden=true
    export function debugName(kind: number): string {
        return isSim() ? "" : hwDebugName(kind);
    }

    //% shim=circuitcubes::debugName
    function hwDebugName(kind: number): string { return ""; }

    // ---- blocks -------------------------------------------------------------

    /**
     * Start scanning for a Circuit Cube ("Tenka" / "Circuit Cube") and connect
     * to the first one found. Returns immediately; use "cube is connected"
     * to wait for the link. Reconnects automatically if the link drops.
     */
    //% blockId=circuitcubes_connect block="connect to Circuit Cube"
    //% weight=100
    export function connect(): void {
        nativeSetNameFilter("");
        nativeConnect();
    }

    /**
     * Connect to a specific Circuit Cube. The cube's Bluetooth name starts with "Tenka"
     * followed by a number; give any part of it, eg: "4130". Use this when more than one
     * cube is switched on. The name is printed in the diagnostics (see README).
     * @param name part of the cube's name, eg: 4130
     */
    //% blockId=circuitcubes_connectnamed block="connect to Circuit Cube named %name"
    //% name.defl="4130"
    //% weight=99
    export function connectToNamed(name: string): void {
        nativeSetNameFilter(name);
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
     * The Bluetooth name of the connected cube, eg: "Tenka4130". Empty when not connected.
     * Handy for showing which cube you are connected to.
     */
    //% blockId=circuitcubes_cubename block="Circuit Cube name"
    //% weight=89
    export function cubeName(): string {
        if (!nativeIsConnected()) return "";
        return isSim() ? "Tenka0000" : hwDebugName(2);
    }

    /**
     * Wait until a Circuit Cube is connected.
     * @param timeoutMs give up after this many milliseconds, 0 waits forever, eg: 0
     */
    //% blockId=circuitcubes_waitconnected block="wait until Circuit Cube is connected || max %timeoutMs ms"
    //% timeoutMs.min=0 timeoutMs.defl=0
    //% weight=95
    export function waitUntilConnected(timeoutMs: number = 0): void {
        const start = control.millis();
        while (!nativeIsConnected()) {
            if (timeoutMs > 0 && control.millis() - start >= timeoutMs) return;
            basic.pause(20);
        }
    }

    // ---- connection events ---------------------------------------------------
    let onConnectedHandler: () => void = null;
    let onDisconnectedHandler: () => void = null;
    let watcherStarted = false;

    // Polls the link state and fires the handlers on every change, so the events
    // also fire for automatic reconnects and for drops.
    function startWatcher(): void {
        if (watcherStarted) return;
        watcherStarted = true;
        control.inBackground(function () {
            let last = nativeIsConnected();
            while (true) {
                basic.pause(50);
                const now = nativeIsConnected();
                if (now == last) continue;
                last = now;
                const h = now ? onConnectedHandler : onDisconnectedHandler;
                if (h) control.inBackground(h);
            }
        });
    }

    /**
     * Run code when a Circuit Cube connects (also after an automatic reconnect).
     */
    //% blockId=circuitcubes_onconnected block="on Circuit Cube connected"
    //% weight=94
    export function onConnected(handler: () => void): void {
        onConnectedHandler = handler;
        startWatcher();
    }

    /**
     * Run code when the Circuit Cube connection is lost or closed.
     */
    //% blockId=circuitcubes_ondisconnected block="on Circuit Cube disconnected"
    //% weight=93
    export function onDisconnected(handler: () => void): void {
        onDisconnectedHandler = handler;
        startWatcher();
    }

    /**
     * Close the connection to the cube and stop scanning.
     * Call "connect to Circuit Cube" to connect again.
     */
    //% blockId=circuitcubes_disconnect block="disconnect Circuit Cube"
    //% weight=85
    export function disconnect(): void {
        nativeDisconnect();
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
     * Run a motor for some time, then stop it.
     * @param port which motor port, eg: CubePort.A
     * @param speed -255 to 255, eg: 255
     * @param ms how long to run in milliseconds, eg: 1000
     */
    //% blockId=circuitcubes_runfor block="run motor %port at speed %speed for %ms ms"
    //% speed.min=-255 speed.max=255 speed.defl=255 ms.min=0 ms.defl=1000
    //% weight=75
    export function runMotorFor(port: CubePort, speed: number, ms: number): void {
        setMotor(port, speed);
        basic.pause(ms);
        setMotor(port, 0);
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

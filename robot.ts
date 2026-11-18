/**
 * Tank-style robot: one motor per track. Speeds are percent (-100..100).
 */
enum TurnDirection {
    //% block="left"
    Left = 0,
    //% block="right"
    Right = 1
}

enum MoveDirection {
    //% block="forward"
    Forward = 0,
    //% block="backward"
    Backward = 1
}

namespace circuitCubes.robot {

    let leftPort = CubePort.A;
    let rightPort = CubePort.B;
    let leftReversed = false;
    let rightReversed = false;
    let curLeft = 0;
    let curRight = 0;

    function clampPct(p: number): number {
        p = Math.round(p);
        return p > 100 ? 100 : (p < -100 ? -100 : p);
    }

    function applyMotors(left: number, right: number): void {
        curLeft = clampPct(left);
        curRight = clampPct(right);
        const l = Math.round(curLeft * 255 / 100) * (leftReversed ? -1 : 1);
        const r = Math.round(curRight * 255 / 100) * (rightReversed ? -1 : 1);
        circuitCubes.setMotor(leftPort, l);
        circuitCubes.setMotor(rightPort, r);
    }

    /**
     * Choose which cube ports drive the left and right track.
     * @param left port of the left motor, eg: CubePort.A
     * @param right port of the right motor, eg: CubePort.B
     */
    //% blockId=circuitcubesrobot_setmotors block="robot left motor %left right motor %right"
    //% group="Setup" subcategory="Robot" weight=100
    export function setMotors(left: CubePort, right: CubePort): void {
        leftPort = left;
        rightPort = right;
    }

    /**
     * Reverse a track's direction. Use this when a motor is mounted mirrored
     * and the robot spins instead of driving straight.
     * @param side which track, eg: TurnDirection.Left
     * @param reversed true to reverse it, eg: true
     */
    //% blockId=circuitcubesrobot_reverse block="robot reverse %side motor %reversed"
    //% reversed.shadow=toggleOnOff reversed.defl=true
    //% group="Setup" subcategory="Robot" weight=90
    export function setReversed(side: TurnDirection, reversed: boolean): void {
        if (side == TurnDirection.Left) leftReversed = reversed; else rightReversed = reversed;
    }

    /**
     * Set the speed of each track independently.
     * @param left left track speed, -100 to 100 percent, eg: 50
     * @param right right track speed, -100 to 100 percent, eg: 50
     */
    //% blockId=circuitcubesrobot_drive block="robot drive left %left\\% right %right\\%"
    //% left.min=-100 left.max=100 left.defl=50 right.min=-100 right.max=100 right.defl=50
    //% group="Drive" subcategory="Robot" weight=80
    export function drive(left: number, right: number): void {
        applyMotors(left, right);
    }

    /**
     * Drive forward or backward at the same speed on both tracks.
     * @param dir forward or backward, eg: MoveDirection.Forward
     * @param speed 0 to 100 percent, eg: 50
     */
    //% blockId=circuitcubesrobot_move block="robot drive %dir at %speed\\%"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% group="Drive" subcategory="Robot" weight=75
    export function move(dir: MoveDirection, speed: number): void {
        const s = Math.abs(speed) * (dir == MoveDirection.Backward ? -1 : 1);
        applyMotors(s, s);
    }

    /**
     * Steer while driving. Direction 0 drives straight, -100 / 100 slows the
     * inner track to a stop for the sharpest curve (the robot arcs around it).
     * Negative steers left, positive steers right. Negative speed reverses.
     * @param direction -100 (left) to 100 (right), eg: 30
     * @param speed -100 to 100 percent, eg: 50
     */
    //% blockId=circuitcubesrobot_steer block="robot steer %direction at %speed\\%"
    //% direction.min=-100 direction.max=100 direction.defl=30
    //% speed.min=-100 speed.max=100 speed.defl=50
    //% group="Drive" subcategory="Robot" weight=70
    export function steer(direction: number, speed: number): void {
        direction = clampPct(direction);
        const inner = speed * (100 - Math.abs(direction)) / 100;
        if (direction < 0) applyMotors(inner, speed);
        else applyMotors(speed, inner);
    }

    /**
     * Turn on the spot: one track forward, the other backward.
     * @param dir left or right, eg: TurnDirection.Left
     * @param speed 0 to 100 percent, eg: 50
     */
    //% blockId=circuitcubesrobot_turn block="robot turn %dir at %speed\\%"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% group="Drive" subcategory="Robot" weight=65
    export function turn(dir: TurnDirection, speed: number): void {
        const s = Math.abs(speed);
        if (dir == TurnDirection.Left) applyMotors(-s, s); else applyMotors(s, -s);
    }

    /**
     * Stop both tracks.
     */
    //% blockId=circuitcubesrobot_stop block="robot stop"
    //% group="Drive" subcategory="Robot" weight=60
    export function stop(): void {
        applyMotors(0, 0);
    }

    /**
     * Drive forward or backward for some time, then stop.
     * @param dir forward or backward, eg: MoveDirection.Forward
     * @param speed 0 to 100 percent, eg: 50
     * @param ms how long in milliseconds, eg: 1000
     */
    //% blockId=circuitcubesrobot_movefor block="robot drive %dir at %speed\\% for %ms ms"
    //% speed.min=0 speed.max=100 speed.defl=50 ms.min=0 ms.defl=1000
    //% group="Timed" subcategory="Robot" weight=50
    export function moveFor(dir: MoveDirection, speed: number, ms: number): void {
        move(dir, speed);
        basic.pause(ms);
        stop();
    }

    /**
     * Turn on the spot for some time, then stop.
     * @param dir left or right, eg: TurnDirection.Left
     * @param speed 0 to 100 percent, eg: 50
     * @param ms how long in milliseconds, eg: 500
     */
    //% blockId=circuitcubesrobot_turnfor block="robot turn %dir at %speed\\% for %ms ms"
    //% speed.min=0 speed.max=100 speed.defl=50 ms.min=0 ms.defl=500
    //% group="Timed" subcategory="Robot" weight=45
    export function turnFor(dir: TurnDirection, speed: number, ms: number): void {
        turn(dir, speed);
        basic.pause(ms);
        stop();
    }

    /**
     * Steer for some time, then stop.
     * @param direction -100 (left) to 100 (right), eg: 30
     * @param speed -100 to 100 percent, eg: 50
     * @param ms how long in milliseconds, eg: 1000
     */
    //% blockId=circuitcubesrobot_steerfor block="robot steer %direction at %speed\\% for %ms ms"
    //% direction.min=-100 direction.max=100 direction.defl=30
    //% speed.min=-100 speed.max=100 speed.defl=50 ms.min=0 ms.defl=1000
    //% group="Timed" subcategory="Robot" weight=40
    export function steerFor(direction: number, speed: number, ms: number): void {
        steer(direction, speed);
        basic.pause(ms);
        stop();
    }

    /**
     * Current left track speed in percent.
     */
    //% blockId=circuitcubesrobot_leftspeed block="robot left speed"
    //% group="Drive" subcategory="Robot" weight=30
    export function leftSpeed(): number { return curLeft; }

    /**
     * Current right track speed in percent.
     */
    //% blockId=circuitcubesrobot_rightspeed block="robot right speed"
    //% group="Drive" subcategory="Robot" weight=29
    export function rightSpeed(): number { return curRight; }
}

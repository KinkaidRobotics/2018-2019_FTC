package org.firstinspires.ftc.teamcode.KarenRobot.General;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Julian on 11/14/2017.
 */

public abstract class SubSystem {

    protected Robot robot;

    public abstract void init();

    public abstract void handle();

    public abstract void stop();

    public SubSystem(Robot robot) {
        this.robot = robot;
    }

    protected HardwareMap hardwareMap() {
        return robot.hardwareMap;
    }

    protected Gamepad gamepad1() {
        return robot.gamepad1;
    }

    protected Gamepad gamepad2() {
        return robot.gamepad2;
    }

    protected Telemetry telemetry() {
        return robot.telemetry;
    }
}

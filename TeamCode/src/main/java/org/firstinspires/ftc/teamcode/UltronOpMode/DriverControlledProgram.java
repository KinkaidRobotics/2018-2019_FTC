package org.firstinspires.ftc.teamcode.UltronOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.Robot;

/**
 * Created by Julian on 11/15/2017.
 */

public abstract class DriverControlledProgram extends OpMode {

    private Robot robot;
    protected boolean twoDrivers;

    protected abstract Robot buildRobot();

    protected void onStart() {
    }

    protected void onUpdate() {
    }

    protected void onStop() {
        robot.stopAllComponents();
    }

    @Override
    public final void init() {
        robot = buildRobot();
        robot.init();
    }

    public final void start() {
        onStart();
    }

    public final void loop() {
        robot.driverControlledUpdate();
        onUpdate();
    }

    public final void stop() {
        onStop();
    }

    protected final Robot getRobot() {return robot;}

}

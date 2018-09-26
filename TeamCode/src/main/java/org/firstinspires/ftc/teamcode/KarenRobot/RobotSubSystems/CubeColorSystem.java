package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

/**
 * Created by Julian on 2/2/2018.
 */

public class CubeColorSystem extends SubSystem {

    private ColorSensor colorSensor;

    public CubeColorSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        colorSensor = hardwareMap().colorSensor.get(Karen.COLOR_SENSOR_KEY);
    }

    @Override
    public void handle() {
        displayValues();
    }

    @Override
    public void stop() {

    }

    public int getRedColor() {
        return colorSensor.red();
    }

    public int getBlueColor() {
        return colorSensor.blue();
    }

    public void displayValues() {
        telemetry().addData("Red", getRedColor());
        telemetry().addData("Blue", getBlueColor());
    }
}

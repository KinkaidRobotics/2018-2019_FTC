package org.firstinspires.ftc.teamcode.UltronRobot.RobotSubSystems;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.Robot;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.Ultron;

/**
 * Created by Julian on 2/2/2018.
 */

public class JewelColorSystem extends SubSystem {

    private ColorSensor colorSensor;

    public JewelColorSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        colorSensor = hardwareMap().colorSensor.get(Ultron.COLOR_SENSOR_KEY);
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

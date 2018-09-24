package org.firstinspires.ftc.teamcode.UltronRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.RobotSubSystems.DogeCVSystem;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.RobotSubSystems.JewelColorSystem;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.RobotSubSystems.VuforiaSystem;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronUtil.SimpleColor;
import org.firstinspires.ftc.teamcode.UltronRobot.Ultron;

/**
 * Created by Seb on 2/1/18.
 */

public class UltronAutoRobot extends Ultron {
    public UltronAutoRobot(OpMode opMode, SimpleColor alliance, Boolean twoDrivers) {
        super(opMode, alliance, twoDrivers);
        putSubSystem("vuforia", new VuforiaSystem(this));
        putSubSystem("dogeCV", new DogeCVSystem(this));
        putSubSystem("jewelColor", new JewelColorSystem(this));
    }
}

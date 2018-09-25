package org.firstinspires.ftc.teamcode.KarenTeleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;
import org.firstinspires.ftc.teamcode.KarenUtil.SimpleColor;
import org.firstinspires.ftc.teamcode.KarenOpMode.DriverControlledProgram;

/**
 * Created by Julian on 11/15/2017.
 */
@TeleOp(name = "Teleop Two")
public class KarenTeleopTwoDriver extends DriverControlledProgram {
    @Override
    protected Robot buildRobot() {
        return new Karen(this, SimpleColor.RED, true);
    }
}

package org.firstinspires.ftc.teamcode.UltronTeleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.Robot;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronUtil.SimpleColor;
import org.firstinspires.ftc.teamcode.UltronOpMode.DriverControlledProgram;
import org.firstinspires.ftc.teamcode.UltronRobot.Ultron;

/**
 * Created by Julian on 11/15/2017.
 */
@TeleOp(name = "Teleop One")
public class UltronTeleoplOneDriver extends DriverControlledProgram {
    @Override
    protected Robot buildRobot() {
        return new Ultron(this, SimpleColor.RED, false);
    }
}

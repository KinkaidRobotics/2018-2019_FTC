package org.firstinspires.ftc.teamcode.KarenUtil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KarenAuto;

/**
 * Created by Julian on 12/16/2017.
 */
@Autonomous(name = "DriveStraightTest", group = "Util")
public class DriveStraightTest extends KarenAuto {
    public DriveStraightTest() {
        super(null);
    }

    @Override
    public void main() {
        driveStraightForward(1000,0.5);
        waitFor(2);
        driveStraightBackwards(-1000, -0.5);
        waitFor(5);
    }
}

package org.firstinspires.ftc.teamcode.KarenUtil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KarenAuto;

/**
 * Created by Julian on 12/17/2017.
 */
@Autonomous(name = "DriveTurnTest", group = "Util")
public class DriveTurnTest extends KarenAuto {
    public DriveTurnTest() {
        super(null);
    }

    @Override
    public void main() {
        driveSystem.driveAngle(0,Math.PI/2,1);
        sleep(1000);
        driveSystem.stopMotors();
    }
}

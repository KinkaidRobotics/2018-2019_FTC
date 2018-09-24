package org.firstinspires.ftc.teamcode.UltronUtil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronAuto;

/**
 * Created by Julian on 12/1/2017.
 */
@Autonomous(name = "GyroTest", group = "Util")
public class GyroTurnTest extends UltronAuto {

    public GyroTurnTest() {
        super(null);
    }

    @Override
    public void main() {
        sensorSystem.updateGyro();
        turn(Math.PI/2);//Turn left 90 degrees
        telemetry.addData("I turned", "left");
        telemetry.update();
        sleep(2000);//Wait 2 seconds
        turn(-Math.PI/2);//Turn right 90 degrees should power be negative here?
        telemetry.addData("I turned", "right");
        telemetry.update();
        sleep(2000);//Wait 2 seconds
    }
}

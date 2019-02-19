package org.firstinspires.ftc.teamcode.UltronAutomodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;

@Autonomous (name = "BlueHangAuton")
public class BlueHangAuton extends KarenAutoBlue {

    @Override
    public void main() {
        //turn test
        sensorSystem.resetGyro();
        autoTurnDegreesRight(90);
        telemetry.addData("Turn Complete", true);
        telemetry.update();
        waitFor(3);
        telemetry.addData("Waiting Complete", true);
        telemetry.update();
        autoTurnDegreesRight(-90);


        //----------------end test------------------------
        // go down
        // move a tiny bit
        // lower the hook - need to get really good at this
        // look at the objects
        // depending on the position of the cube
        // go knock out the yellow thing
        // try to park
    }
}

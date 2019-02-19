package org.firstinspires.ftc.teamcode.UltronAutomodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;

@Autonomous (name = "BlueHangAutonPark")
public class BlueHangAutonPark extends KarenAutoBlue {

    @Override
    public void main() {
        // go down
        autoGoToLiftPos(LiftSystem.LiftState.UP, 1);
        waitFor(1);
        //this is where we would identify the gold position
        // move a tiny bit
        driveTime(-.25, .75);
        waitFor(1);
        // lower the hook - need to get really good at this
        autoGoToLiftPos(LiftSystem.LiftState.DOWN, 1);
        // if using vision now go to one of the three positions, then park
        // else just park
        autoTurnDegreesRight(-90);
        telemetry.addData("Done Turning", true);
        telemetry.update();
        driveTime(1, 3);
    }
}

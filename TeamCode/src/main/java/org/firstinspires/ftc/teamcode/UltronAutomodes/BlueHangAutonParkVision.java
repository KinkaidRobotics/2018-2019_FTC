package org.firstinspires.ftc.teamcode.UltronAutomodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.VisionSystem;

@Autonomous (name = "BlueHangAutonParkVision")
public class BlueHangAutonParkVision extends KarenAutoBlue {

    @Override
    public void main() {
        //------------------------------------------
        // According to CAD, for camera view
        // for left turn .45 rads left
        // for center turn .07 rads left
        // for right turn .32 rads right
        // so in degrees:
        // left:     -25.78
        // center:   -4.01
        // right    18.33
        // measuring
        // left:  -29
        // center: -6
        // right: 20
        // -------------------------------------------------
        // for robot turn:
        // left: -112
        // center: -89
        // right: -67
        //------------------------------------------
        VisionSystem.GoldPos goldPos;
        // go down
        waitFor(2);
        autoGoToLiftPos(LiftSystem.LiftState.UP, .5);
        waitFor(.25);
        // unhook the hook
        // ToDo: calibrate this distance and use AutoDriveWithEncoder(ticks)
        driveTime(-.25, .75);
        waitFor(.25);

        // lower the hook - need to get really good at this
        autoGoToLiftPos(LiftSystem.LiftState.DOWN, 1);
        waitFor(.25);
        // makes sure that the hook is actually lowered

        // goes back to right under the hook, this should also use the other method
        driveTime(.25, .75);

        waitFor(.25);
        // let everything get settled

        // turn to view center
        autoTurnDegreesRight(-6);
        waitFor(.25);

        if (visionSystem.getDetector().isFound() && visionSystem.isCentered()) {
            // if the gold one is in the center, then set it and be done
            goldPos = VisionSystem.GoldPos.CENTER;
        } else {
            // measured right degrees - measured center degrees
            autoTurnDegreesRight(26);

            waitFor(.25);

            if (visionSystem.getDetector().isFound() && visionSystem.isCentered()) {
                goldPos = VisionSystem.GoldPos.RIGHT;
            } else {
                // measured left degrees-measured right degrees
                autoTurnDegreesRight(-49);

                waitFor(.25);

                if (visionSystem.getDetector().isFound() && visionSystem.isCentered()) {
                    goldPos = VisionSystem.GoldPos.LEFT;
                } else {
                    goldPos = VisionSystem.GoldPos.NOT_FOUND;
                }
            }
        }
        telemetry.addData("Gold Pos", goldPos);
        telemetry.update();
        // this is just to give time to see the position
        waitFor(.25);

        // if using vision now go to one of the three positions
        switch (goldPos) {
            case LEFT:
                autoTurnDegreesRight(-83);
                // do some stuff
            case CENTER:
                autoTurnDegreesRight(-83);
                // do some other stuff
            case RIGHT:
                autoTurnDegreesRight(-87);
                // do even more stuff
            case NOT_FOUND:
                // means was in left position
                autoTurnDegreesRight(-83);
                // do something
        }
        driveTime(1, 2);
    }
}

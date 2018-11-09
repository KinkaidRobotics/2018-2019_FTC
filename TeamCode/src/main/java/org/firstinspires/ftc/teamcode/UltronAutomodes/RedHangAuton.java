package org.firstinspires.ftc.teamcode.UltronAutomodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;

@Autonomous (name = "RedHangAuton")
public class RedHangAuton extends KarenAutoRed {

    @Override
    public void main() {
        liftSystem.pullPin();

        //Go to lift up position, lowering robot
        autoGoToLiftPos(LiftSystem.LiftState.UP, .25);

        // Unhook/move out of the way
        driveTime(-.75, 1);

        //After moving the robot lower the lift
        autoGoToLiftPos(LiftSystem.LiftState.DOWN,.75);


    }

}

package org.firstinspires.ftc.teamcode.UltronAutomodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;

@Autonomous (name = "RedHangAutonNoRetract")
public class RedHangAutonNoRetract extends KarenAutoRed {

    @Override
    public void main() {

        //Go to lift up position, lowering robot
        //waitFor(10);//optional, use this to make sure alliance partner is out of the way
        autoGoToLiftPos(LiftSystem.LiftState.UP, 1);
        waitFor(1);
        driveTime(-.25, .5);
//        autoGoToLiftPos(LiftSystem.LiftState.DOWN, 1);


//        waitFor(1);
//        driveTime(.25, .5);
//        waitFor(1);
//        autoGoToLiftPos(LiftSystem.LiftState.DOWN,1);

//        // Unhook/move out of the way
//        driveTime(-.75, .5);
//
//        //After moving the robot lower the lift
//        autoGoToLiftPos(LiftSystem.LiftState.DOWN,.75);


    }

}

package org.firstinspires.ftc.teamcode.UltronAutomodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;

@Autonomous (name = "BlueHangAuton")
public class BlueHangAuton extends KarenAutoBlue {

    @Override
    public void main() {
        //Go to lift up position, lowering robot
        autoGoToLiftPos(LiftSystem.LiftState.UP, .25);

//        // Unhook/move out of the way
//        driveTime(-.75, .5);
//
//        //After moving the robot lower the lift
//        autoGoToLiftPos(LiftSystem.LiftState.DOWN,.75);
    }

}

package org.firstinspires.ftc.teamcode.UltronRobot.RobotSubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.Robot;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.Ultron;

/**
 * Created by Julian on 11/16/2017.
 */
public class JewelArmSystem extends SubSystem {
    private Servo topRight, lowerRight, topLeft, lowerLeft;

    public JewelArmSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        topRight = hardwareMap().servo.get(Ultron.RIGHT_TOP_JEWEL_SERVO_KEY);
        lowerRight = hardwareMap().servo.get(Ultron.RIGHT_LOWER_JEWEL_SERVO_KEY);
//        topLeft = hardwareMap().servo.get(Ultron.LEFT_TOP_JEWEL_SERVO_KEY);
//        lowerLeft = hardwareMap().servo.get(Ultron.LEFT_LOWER_JEWEL_SERVO_KEY);

        rightUp();
//        leftUp();
    }

    @Override
    public void handle() {
        if (false/*robot.TWO_DRIVERS*/) {
            if (gamepad2().a) {
                rightDown();
//                leftDown();
            }
            if (gamepad2().y) {
                rightUp();
//                leftUp();
            }
        }else {
            if (gamepad1().a) {
                rightDown();
//                leftDown();
            }
            if (gamepad1().y) {
                rightUp();
//                leftUp();
            }
        }
    }

    @Override
    public void stop() {

    }

    public void rightDown() {
        topRight.setPosition(Ultron.RIGHT_TOP_SERVO_DOWN);
        lowerRight.setPosition(Ultron.RIGHT_LOWER_SERVO_DOWN);
    }

    public void rightUp() {
        topRight.setPosition(Ultron.RIGHT_TOP_SERVO_UP);
        lowerRight.setPosition(Ultron.RIGHT_LOWER_SERVO_UP);
    }

//    public void leftDown() {
//        topLeft.setPosition(Ultron.LEFT_TOP_SERVO_DOWN);
//        lowerLeft.setPosition(Ultron.LEFT_LOWER_SERVO_DOWN);
//    }
//
//    public void leftUp() {
//        topLeft.setPosition(Ultron.LEFT_TOP_SERVO_UP);
//        lowerLeft.setPosition(Ultron.LEFT_LOWER_SERVO_UP);
//    }
}

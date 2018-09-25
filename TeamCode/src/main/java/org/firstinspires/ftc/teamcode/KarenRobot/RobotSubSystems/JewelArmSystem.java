package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

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
        topRight = hardwareMap().servo.get(Karen.RIGHT_TOP_JEWEL_SERVO_KEY);
        lowerRight = hardwareMap().servo.get(Karen.RIGHT_LOWER_JEWEL_SERVO_KEY);
//        topLeft = hardwareMap().servo.get(Karen.LEFT_TOP_JEWEL_SERVO_KEY);
//        lowerLeft = hardwareMap().servo.get(Karen.LEFT_LOWER_JEWEL_SERVO_KEY);

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
        topRight.setPosition(Karen.RIGHT_TOP_SERVO_DOWN);
        lowerRight.setPosition(Karen.RIGHT_LOWER_SERVO_DOWN);
    }

    public void rightUp() {
        topRight.setPosition(Karen.RIGHT_TOP_SERVO_UP);
        lowerRight.setPosition(Karen.RIGHT_LOWER_SERVO_UP);
    }

//    public void leftDown() {
//        topLeft.setPosition(Karen.LEFT_TOP_SERVO_DOWN);
//        lowerLeft.setPosition(Karen.LEFT_LOWER_SERVO_DOWN);
//    }
//
//    public void leftUp() {
//        topLeft.setPosition(Karen.LEFT_TOP_SERVO_UP);
//        lowerLeft.setPosition(Karen.LEFT_LOWER_SERVO_UP);
//    }
}

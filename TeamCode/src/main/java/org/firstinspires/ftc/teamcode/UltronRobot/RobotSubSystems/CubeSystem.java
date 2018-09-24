package org.firstinspires.ftc.teamcode.UltronRobot.RobotSubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.Robot;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.Ultron;

/**
 * Created by Julian on 11/16/2017.
 */

public class CubeSystem extends SubSystem {

    private Servo rightTopServo;
    private Servo leftTopServo;
    private Servo rightLowerServo;
    private Servo leftLowerServo;

    public CubeSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        rightTopServo = hardwareMap().servo.get(Ultron.RIGHT_TOP_SERVO_KEY);
        leftTopServo = hardwareMap().servo.get(Ultron.LEFT_TOP_SERVO_KEY);

        openTop();
    }

    @Override
    public void handle() {
        if (false/*robot.TWO_DRIVERS*/) {
            if (gamepad2().b) {
                openTop();
            }

            if (gamepad2().x) {
                closeTop();
            }

        }else {
            if (gamepad1().b) {
                openTop();
            }

            if (gamepad1().x) {
                closeTop();
            }

        }
    }

    @Override
    public void stop() {
        openTop();
    }

    public void openTop() {
        rightTopServo.setPosition(Ultron.RIGHT_TOP_SERVO_OPEN);
        leftTopServo.setPosition(Ultron.LEFT_TOP_SERVO_OPEN);
    }

    public void closeTop() {
        rightTopServo.setPosition(Ultron.RIGHT_TOP_SERVO_CLOSED);
        leftTopServo.setPosition(Ultron.LEFT_TOP_SERVO_CLOSED);
    }
}

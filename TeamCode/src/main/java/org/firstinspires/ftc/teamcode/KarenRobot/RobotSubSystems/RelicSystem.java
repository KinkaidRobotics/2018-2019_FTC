package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

/**
 * Created by Julian on 2/3/2018.
 */

public class RelicSystem extends SubSystem {

    private Servo clawServo;
    private Servo relicRotateServo;
    private DcMotor relicExtension;

    public RelicSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        clawServo = hardwareMap().servo.get(Karen.CLAW_SERVO_KEY);
        relicRotateServo = hardwareMap().servo.get(Karen.RELIC_ROTATE_SERVO_KEY);
        relicExtension = hardwareMap().dcMotor.get(Karen.RELIC_EXTENSION_KEY);
//        releaseRelic();
//        lowerRelic();
    }

    @Override
    public void handle() {
        relicExtension.setPower(-gamepad2().right_stick_y);
        if (gamepad2().a) {
            lowerRelic();
        } else if (gamepad2().y) {
            raiseRelic();
        }
        if (gamepad2().dpad_down) {
            grabRelic();
        } else if (gamepad2().dpad_up) {
            releaseRelic();
        }
    }

    @Override
    public void stop() {

    }

    public void grabRelic() {
        clawServo.setPosition(Karen.RELIC_CLAW_CLOSE);
    }

    public void releaseRelic() {
        clawServo.setPosition(Karen.RELIC_CLAW_OPEN);
    }

    public void raiseRelic() {
        relicRotateServo.setPosition(Karen.RELIC_UP);
    }

    public void lowerRelic() {
        relicRotateServo.setPosition(Karen.RELIC_DOWN);
    }
}

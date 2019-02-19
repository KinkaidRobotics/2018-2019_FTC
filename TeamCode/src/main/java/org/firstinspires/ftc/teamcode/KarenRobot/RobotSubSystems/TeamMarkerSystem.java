package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

/**
 * Created by Julian on 11/14/2017.
 */

public class TeamMarkerSystem extends SubSystem {
    private double closedPos = 0; // This needs to be calibrated
    private double openPos = .5; // This needs to be calibrated
    private Servo teamMarkerServo;

    public TeamMarkerSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        teamMarkerServo = hardwareMap().servo.get(Karen.TEAM_MARKER_SERVO_KEY);
    }

    @Override
    // Look at this stuff
    public void handle() {
        if (gamepad1().x) {
            openServo();
        }
        if (gamepad1().b) {
            closeServo();
        }
    }

    @Override
    public void stop() {
        closeServo();
    }

    public void openServo() {
        teamMarkerServo.setPosition(openPos);
    }

    public void closeServo() {
        teamMarkerServo.setPosition(closedPos);
    }


}

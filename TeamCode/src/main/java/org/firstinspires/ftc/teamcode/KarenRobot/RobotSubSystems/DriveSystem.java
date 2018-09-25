package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

/**
 * Created by Julian on 11/14/2017.
 */

public class DriveSystem extends SubSystem {

    private DcMotor frontRight, frontLeft, rearRight, rearLeft;
    private SensorSystem sensorSystem;
    private boolean slow = false;

    public DriveSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        this.sensorSystem = robot.getSubSystem(SensorSystem.class);

        frontLeft = hardwareMap().dcMotor.get(Karen.DRIVE_FL_KEY);
        rearLeft = hardwareMap().dcMotor.get(Karen.DRIVE_RL_KEY);
        frontRight = hardwareMap().dcMotor.get(Karen.DRIVE_FR_KEY);
        rearRight = hardwareMap().dcMotor.get(Karen.DRIVE_RR_KEY);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();
        floatMode();
    }

    @Override
    public void handle() {
//        if (gamepad1().right_bumper) {
//            slow = true;
//        } else {
//            slow = false;
//        }
        if (gamepad1().right_bumper) {
            mecanumTrigPlayer();
        } else if (gamepad1().left_bumper) {
            mecanumNoTrig();
        } else {
            mecanumTrigRobot();
        }
        if (gamepad1().left_stick_button) {
            resetEncoders();
        }
        displayPositions();
    }

    @Override
    public void stop() {
        stopMotors();
    }

    /**
     * floatMode will set motors to float when motor power is set to 0
     */
    public void floatMode() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * brakeMode will set motors to brake when motor power is set to 0
     */
    public void brakeMode() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * displayPositions will display the current encoder values
     */
    public void displayPositions() {
        telemetry().addData("Front Right", frontRight.getCurrentPosition());
        telemetry().addData("Front Left", frontLeft.getCurrentPosition());
        telemetry().addData("Rear Right", rearRight.getCurrentPosition());
        telemetry().addData("Rear Left", rearLeft.getCurrentPosition());
    }

    /**
     * stopMotors will set motors to brake and then sets motor power to 0
     */
    public void stopMotors() {
        brakeMode();
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    /**
     * resets encoder values and returns to voltage mode
     */
    public void resetEncoders() {
        modeReset();
        modeVoltage();
    }

    /**
     * modeVoltage sets all drive motors to RUN_WITHOUT_ENCODERs
     */
    public void modeVoltage() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * modeSpeed sets all drive motors to RUN_USING_ENCODERs
     */
    public void modeSpeed() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * modeReset resets all drive encoder values
     */
    public void modeReset() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * driveForward drives directly forward at the given power
     * @param power
     */
    public void driveForward(double power) {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
    }

    /**
     * mecanumTrigPlayer takes joystick values and sets robot to drive at an angle based on it
     * the drive is based on player perspective, not robot perspective
     */
    public void mecanumTrigPlayer() {
        //See http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf
        double r = (Math.hypot(gamepad1().left_stick_x, -gamepad1().left_stick_y))/Math.sqrt(2);
        double robotAngle = Math.atan2(-gamepad1().left_stick_x,-gamepad1().left_stick_y)  - sensorSystem.getYaw() + Math.PI / 4;
        double rightX = gamepad1().right_stick_x;

        driveAngle(r, robotAngle, rightX);
    }

    /**
     * mecanumTrigRobot takes joystick values and sets robot to drive at an angle based on it
     * the drive is based on robot perspective, not player perspective
     */
    public void mecanumTrigRobot() {
        double r = (Math.hypot(gamepad1().left_stick_x, -gamepad1().left_stick_y))/Math.sqrt(2);
        double robotAngle = Math.atan2(-gamepad1().left_stick_x,-gamepad1().left_stick_y) + Math.PI / 4;
        double rightX = gamepad1().right_stick_x;

        driveAngle(r, robotAngle, rightX);
    }

    /**
     * mecanumNoTrig will take joystick values and then set robot power accordingly, this does not
     * use any trig and also does not factor robot orientation into it
     */
    public void mecanumNoTrig() {
        double Ch1 = gamepad1().right_stick_x;
        double Ch3 = -gamepad1().left_stick_y;
        double Ch4 = -gamepad1().left_stick_x;

        double frontLeftPower = Ch3 + Ch1 + Ch4;
        double rearLeftPower = Ch3 + Ch1 - Ch4;
        double frontRightPower = Ch3 - Ch1 - Ch4;
        double rearRightPower = Ch3 - Ch1 + Ch4;

        setPower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    /**
     * setPower sets the power of all the motors to their set power and will clip values if they
     * are greater than 1 or less than -1
     * @param frontLeftPower should range from about 1 to -1, but can differ
     * @param frontRightPower should range from about 1 to -1, but can differ
     * @param rearLeftPower should range from about 1 to -1, but can differ
     * @param rearRightPower should range from about 1 to -1, but can differ
     */
    public void setPower(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0) ;
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0) ;
        rearLeftPower = Range.clip(rearLeftPower, -1.0, 1.0) ;
        rearRightPower = Range.clip(rearRightPower, -1.0, 1.0) ;

        if (slow) {
            frontLeft.setPower(frontLeftPower / 2.0);
            frontRight.setPower(frontRightPower / 2.0);
            rearLeft.setPower(rearLeftPower / 2.0);
            rearRight.setPower(rearRightPower / 2.0);
        } else {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            rearLeft.setPower(rearLeftPower);
            rearRight.setPower(rearRightPower);
        }
    }

    /**
     * driveAngle will set robot to go in target direction at target power and will turn at turn power
     * @param inPower ranges between -1 and 1 and is the strength of the robot's movement
     * @param robotAngle takes a value in radians, this should factor in robot position
     * @param turnAmount takes a value between -1 and 1 which should give power of turn
     */
    public void driveAngle(double inPower, double robotAngle, double turnAmount) {
        double frontLeftPower = inPower * Math.sin(robotAngle) + turnAmount;
        double frontRightPower = inPower * Math.cos(robotAngle) - turnAmount;
        double rearLeftPower = inPower * Math.cos(robotAngle) + turnAmount;
        double rearRightPower = inPower * Math.sin(robotAngle) - turnAmount;

        double[] powerList = new double[4];
        double absMax = 0;
        powerList[0] = frontLeftPower;
        powerList[1] = frontRightPower;
        powerList[2] = rearLeftPower;
        powerList[3] = rearRightPower;

        for (double power:powerList){
            if (Math.abs(power) > absMax)
                absMax = Math.abs(power);
        }

        if (absMax > 1) {
            for (int i = 0; i < powerList.length; i++) {
                powerList[i] /= absMax;
            }
        }

        frontLeftPower = powerList[0];
        frontRightPower = powerList[1];
        rearLeftPower = powerList[2];
        rearRightPower = powerList[3];

        setPower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    /**
     * turns a number of radians at a set power utilizing the gyro sensor
     * @param radians positive for right, negative for left, should range between -2pi and 2pi
     * @param power positive values and will be made positive if not, should rage between 0 and 1
     * @throws InterruptedException has a while statement that doesn't check if OpMode is active and could get mad
     */
    public void gyroTurn(double radians, double power) throws InterruptedException {
        power = Math.abs(power);
        double direction = Math.signum(radians);
        radians = radians%(Math.PI*2);
        double initialAngle = sensorSystem.getYaw();
        driveAngle(0,Math.PI/2, power*direction);
        while (Math.abs(initialAngle-sensorSystem.getYaw()) < Math.abs(radians)){sensorSystem.updateGyro();}
        stopMotors();
        resetEncoders();
    }

    public int getRightEncoderValue() {
        int encoderPosition;
        encoderPosition = frontRight.getCurrentPosition();
        return encoderPosition;
    }

    public int getLeftEncoderValue() {
        int encoderPosition;
        encoderPosition = frontRight.getCurrentPosition();
        return encoderPosition;
    }

    public void setTurnPower(double rightPower, double leftPower) {
        frontRight.setPower(rightPower);
        rearRight.setPower(rightPower);
        frontLeft.setPower(leftPower);
        rearLeft.setPower(leftPower);
    }

    public DcMotor getFrontRight() {
        return frontRight;
    }

    public DcMotor getFrontLeft() {
        return frontLeft;
    }

    public DcMotor getRearRight() {
        return rearRight;
    }

    public DcMotor getRearLeft() {
        return rearLeft;
    }
}

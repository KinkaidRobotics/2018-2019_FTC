package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

/**
 * Created by Julian on 11/14/2017.
 */

public class DriveSystem extends SubSystem {
    private int NUM_MOTORS = 2;
    private DcMotor dcMotors[] = new DcMotor[NUM_MOTORS]; // right, left
    private boolean slow = false;
    private boolean lastBumperVal = false;

    public DriveSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        // this.sensorSystem = robot.getSubSystem(SensorSystem.class);

        dcMotors[0] = hardwareMap().dcMotor.get(Karen.DRIVE_RIGHT_KEY);
        dcMotors[1] = hardwareMap().dcMotor.get(Karen.DRIVE_LEFT_KEY);

        for (int i = 0; i < dcMotors.length; i++) {
            if (i % 2 == 0) {
                dcMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                dcMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }

        resetEncoders();
        floatMode();
    }

    @Override
    // Look at this stuff
    public void handle() {
        if (gamepad1().right_bumper && !lastBumperVal) {
            slow = !slow;
        }
        setPower(-gamepad1().left_stick_y, -gamepad1().right_stick_y);
        if (gamepad1().left_stick_button) {
            resetEncoders();
        }
        displayPositions();
        lastBumperVal = gamepad1().right_bumper;
    }

    @Override
    public void stop() {
        stopMotors();
    }

    /**
     * floatMode will set motors to float when motor power is set to 0
     */
    public void floatMode() {
        for (DcMotor motor: dcMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     * brakeMode will set motors to brake when motor power is set to 0
     */
    public void brakeMode() {
        for (DcMotor motor: dcMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * displayPositions will display the current encoder values
     */
    public void displayPositions() {
        for (int i = 0; i < dcMotors.length; i++) {
            telemetry().addData("Motor "+ i + " ", dcMotors[i].getCurrentPosition());
        }
    }

    /**
     * stopMotors will set motors to brake and then sets motor power to 0
     */
    public void stopMotors() {
        brakeMode();
        setPower(0);
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
        for (DcMotor motor : dcMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * modeSpeed sets all drive motors to RUN_USING_ENCODERs
     */
    public void modeSpeed() {
        for (DcMotor motor : dcMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * modeReset resets all drive encoder values
     */
    public void modeReset() {
        for (DcMotor motor : dcMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * setPower sets the power of all the motors to their set power and will clip values if they
     * are greater than 1 or less than -1
     * @param leftPower should range from about 1 to -1, but can differ
     * @param rightPower should range from about 1 to -1, but can differ
     */
    public void setPower(double leftPower, double rightPower) {
        leftPower = Range.clip(leftPower, -1.0, 1.0) ;
        rightPower = Range.clip(rightPower, -1.0, 1.0) ;

        if (slow) {
            for (int i = 0; i < dcMotors.length; i++) {
                if (i % 2 == 0) { // is right
                    dcMotors[i].setPower(rightPower / 2.0);
                } else {
                    dcMotors[i].setPower(leftPower / 2.0);
                }
            }
        } else {
            for (int i = 0; i < dcMotors.length; i++) {
                if (i % 2 == 0) { // is right
                    dcMotors[i].setPower(rightPower);
                } else {
                    dcMotors[i].setPower(leftPower);
                }
            }
        }
    }

    /**
     * setPower sets the power of all the motors to their set power and will clip values if they
     * are greater than 1 or less than -1
     * @param power should range from about 1 to -1, but can differ
     */
    public void setPower(double power) {
        power = Range.clip(power, -1.0, 1.0) ;

        if (slow) {
            for (DcMotor motor : dcMotors) {
                motor.setPower(power / 2.0);
            }
        } else {
            for (DcMotor motor : dcMotors) {
                motor.setPower(power);
            }
        }
    }

//    /**
//     * turns a number of radians at a set power utilizing the gyro sensor
//     * @param radians positive for right, negative for left, should range between -2pi and 2pi
//     * @param power positive values and will be made positive if not, should rage between 0 and 1
//     * @throws InterruptedException has a while statement that doesn't check if OpMode is active and could get mad
//     */
//    public void gyroTurn(double radians, double power) throws InterruptedException {
//        power = Math.abs(power);
//        double direction = Math.signum(radians);
//        radians = radians%(Math.PI*2);
//        double initialAngle = sensorSystem.getYaw();
//        setPower(power, -power); //ToDo: Check direction of this, this could go in reverse
//        while (Math.abs(initialAngle-sensorSystem.getYaw()) < Math.abs(radians)){sensorSystem.updateGyro();}
//        stopMotors();
//        resetEncoders();
//    }

    public int getRightEncoderValue() {
        int encoderPosition;
        encoderPosition = dcMotors[0].getCurrentPosition();
        return encoderPosition;
    }

    public int getLeftEncoderValue() {
        int encoderPosition;
        encoderPosition = dcMotors[1].getCurrentPosition();
        return encoderPosition;
    }
}

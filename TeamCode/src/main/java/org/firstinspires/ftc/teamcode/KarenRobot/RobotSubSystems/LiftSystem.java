package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

/**
 * Created by Julian on 11/9/2018
 */

public class LiftSystem extends SubSystem {
    private DcMotor liftMotorUp, liftMotorDown;
    private boolean modeManual = false;
    private boolean wasA = false;
    private int pullTime = 500; // Time is in milliseconds
    private boolean pullingPin = false;
    private boolean wasB = false;
    private long stopTime = 0;

    public enum LiftState {
        UP,
        DOWN
    }

    private LiftState liftState;

    public LiftSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        // this.sensorSystem = robot.getSubSystem(SensorSystem.class);

        liftMotorUp = hardwareMap().dcMotor.get(Karen.LIFT_MOTOR_UP_KEY);
        liftMotorDown = hardwareMap().dcMotor.get(Karen.LIFT_MOTOR_DOWN_KEY);

        liftMotorUp.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorDown.setDirection(DcMotorSimple.Direction.FORWARD);

        liftState = LiftState.DOWN;

        brakeMode();
        resetEncoders();
        modeVoltage();
    }

    @Override
    public void handle() {
        // Change target lift state if needed
        if (gamepad1().dpad_up) {
            liftState = LiftState.UP;
        } else if (gamepad1().dpad_down) {
            liftState = LiftState.DOWN;
        }

        // decide whether or not to use manual controls
        if (gamepad1().a && !wasA) {
            modeManual = !modeManual;
        }

        //move the lift to the desired location
        if (!modeManual) {
            updateLiftPos();
        } else {
            if (gamepad1().right_trigger > .1) {
                modeVoltage();
                setPower(gamepad1().right_trigger);
            } else if (gamepad1().left_trigger > .1) {
                setPower(-gamepad1().left_trigger);
            } else {
                setPower(0);
            }
        }

        //Have the option to reset the encoders
        if (gamepad1().left_stick_button) {
            resetEncoders();
        }

        //display the position
        displayPosition();

        //update previous state variables
        wasA = gamepad1().a;
        wasB = gamepad1().b;
    }

    @Override
    public void stop() {
        stopMotors();
    }

    /**
     * floatMode will set motors to float when motor power is set to 0
     */
    public void floatMode() {
        liftMotorUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotorDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * brakeMode will set motors to brake when motor power is set to 0
     */
    public void brakeMode() {
        liftMotorUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * displayPosition will display the current encoder values
     */
    public void displayPosition() {
        telemetry().addData("LiftMotorUp: ", liftMotorUp.getCurrentPosition());
        telemetry().addData("LiftMotorDown: ", liftMotorDown.getCurrentPosition());
        telemetry().addData("Manual: ", modeManual);
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
        liftMotorUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * modeSpeed sets all drive motors to RUN_USING_ENCODERs
     */
    public void modeSpeed() {
        liftMotorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * modeReset resets all drive encoder values
     */
    public void modeReset() {
        liftMotorUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * setPower sets the power of all the motors to their set power and will clip values if they
     * are greater than 1 or less than -1
     *
     * @param power  should range from about 1 to -1, but can differ
     */
    public void setPower(double power) {
        power = Range.clip(power, -1,1);
        liftMotorUp.setPower(power);
        liftMotorDown.setPower(power);
    }

    public void goToTargetLiftPos(int position) {
        int currentPosition = liftMotorUp.getCurrentPosition();
        int difference = currentPosition - position;
        liftMotorUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorUp.setTargetPosition(position);
        liftMotorDown.setTargetPosition(position);
        if (liftMotorUp.isBusy() || liftMotorDown.isBusy()) {
            setPower(1);
        } else {
            setPower(0);
        }
    }

    public void goToTargetLiftPos(int position, double power) {
        int currentPosition = liftMotorUp.getCurrentPosition();
        int difference = currentPosition - position;
        liftMotorUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorUp.setTargetPosition(position);
        liftMotorDown.setTargetPosition(position);
        if (liftMotorUp.isBusy() || liftMotorDown.isBusy()) {
            setPower(power);
        } else {
            setPower(0);
        }
    }

    public void updateLiftPos() {
        switch (liftState) {
            case UP:
                goToTargetLiftPos(Karen.UP_HEIGHT);
                break;
            case DOWN:
                goToTargetLiftPos(Karen.DOWN_HEIGHT);
                break;
        }
    }

    public void setLiftState(LiftState inLiftState) {
        liftState = inLiftState;
    }

    public DcMotor getLiftMotorUp() {
        return liftMotorUp;
    }

    public DcMotor getLiftMotorDown() {
        return liftMotorDown;
    }

    public int getLiftPos() {
        return liftMotorUp.getCurrentPosition();
    }
}

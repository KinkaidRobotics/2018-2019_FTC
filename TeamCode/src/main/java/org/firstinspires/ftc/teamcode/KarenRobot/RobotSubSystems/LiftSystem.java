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
    private int NUM_MOTORS = 2;
    private DcMotor liftMotor, releaseMotor;
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

        liftMotor = hardwareMap().dcMotor.get(Karen.LIFT_MOTOR_KEY);
        releaseMotor = hardwareMap().dcMotor.get(Karen.RELEASE_MOTOR_KEY);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        releaseMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        liftState = LiftState.DOWN;

        brakeMode();
        resetEncoders();
        modeVoltage();
    }

    @Override
    // Look at this stuff
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

        // have the option to pull the pin/continue pulling the pin
        pullPin();

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
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * brakeMode will set motors to brake when motor power is set to 0
     */
    public void brakeMode() {
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * displayPosition will display the current encoder values
     */
    public void displayPosition() {
        telemetry().addData("LiftMotor: ", liftMotor.getCurrentPosition());
        telemetry().addData("ReleaseMotor: ", releaseMotor.getCurrentPosition());
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
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * modeSpeed sets all drive motors to RUN_USING_ENCODERs
     */
    public void modeSpeed() {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * modeReset resets all drive encoder values
     */
    public void modeReset() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * setPower sets the power of all the motors to their set power and will clip values if they
     * are greater than 1 or less than -1
     *
     * @param power  should range from about 1 to -1, but can differ
     */
    public void setPower(double power) {
        power = Range.clip(power, -1,1);
        liftMotor.setPower(power);
    }

    public void goToTargetLiftPos(int position) {
        int currentPosition = liftMotor.getCurrentPosition();
        int difference = currentPosition - position;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(position);
        if (liftMotor.isBusy()) {
            setPower(0.75);
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

    public void pullPin() {
        if (gamepad1().b && !wasB) {
            stopTime = System.currentTimeMillis() + pullTime;
        }

        if (System.currentTimeMillis() < stopTime && pullingPin) {
            releaseMotor.setPower(1);
        } else {
            releaseMotor.setPower(0);
        }
    }
}

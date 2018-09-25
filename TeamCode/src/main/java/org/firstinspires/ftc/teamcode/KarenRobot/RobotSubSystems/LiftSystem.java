package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

/**
 * Created by Julian on 11/15/2017.
 */

public class LiftSystem extends SubSystem {
    private DcMotor rightLiftMotor;

    private boolean dPadWasUp = false;
    private boolean dPadWasDown = false;
    private boolean notManual = true;
    private double liftPower = 0;
    private boolean resettingLift = false;
    private long resetEndTime = 0;

    public enum LiftState{
        ZERO_CUBE_HEIGHT,
        HALF_CUBE_HEIGHT,
        ONE_CUBE_HEIGHT,
        TWO_CUBE_HEIGHT,
        THREE_CUBE_HEIGHT
    }

    private LiftState liftState;


    public LiftSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        rightLiftMotor = hardwareMap().dcMotor.get(Karen.LIFT_R_WINCH_KEY);

        rightLiftMotor.setDirection(DcMotor.Direction.FORWARD);

        liftState = LiftState.ZERO_CUBE_HEIGHT;

        liftBrakeMode();
        resetEncoders();
        setLiftVoltageMode();
    }

    @Override
    public void handle() {
        if (false/*robot.TWO_DRIVERS*/) {
            handleChangeInDPad(gamepad2());
            //manualControls(gamepad2());
//            resetLift(gamepad1());
        }else {
            handleChangeInDPad(gamepad1());
            //manualControls(gamepad1());
//            resetLift(gamepad1());
        }
//        if (!resettingLift) {
            goToState(liftState);
//        }

        telemetry().addData("Lift Position: ", getLiftPosition());
    }

    @Override
    public void stop() {
        liftFloatMode();
        setLiftPower(0);
    }

    public void resetLift(Gamepad gamepad) {
        if (gamepad.left_stick_button) {
            resettingLift = true;
            resetEndTime = System.currentTimeMillis() + 500;
        }
        if (System.currentTimeMillis()< resetEndTime) {
            rightLiftMotor.setPower(-1);
        }
        if (resettingLift && System.currentTimeMillis() > resetEndTime) {
            resetEncoders();
            setLiftVoltageMode();
            resettingLift = false;
        }
    }

    public void liftFloatMode() {
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void liftBrakeMode() {
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setLiftPower(double power) {
        rightLiftMotor.setPower(power);
    }

    public void setLiftVoltageMode() {
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftEncoderMode() {
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders() {
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goToTargetLiftPos(int position) {
        int currentPosition = rightLiftMotor.getCurrentPosition();
        int difference = currentPosition - position;
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setTargetPosition(position);
        if (rightLiftMotor.isBusy()) {
            setLiftPower(0.75);
        } else {
            setLiftPower(0);
        }
    }

    public void handleChangeInDPad(Gamepad gamepad) {
        Gamepad gamepadToUse = gamepad;

        if (gamepadToUse.dpad_up && !dPadWasUp) {
            switch (liftState){
                case ZERO_CUBE_HEIGHT:
                    liftState = LiftState.HALF_CUBE_HEIGHT;
                    break;
                case HALF_CUBE_HEIGHT:
                    liftState = LiftState.ONE_CUBE_HEIGHT;
                    break;
                case TWO_CUBE_HEIGHT:
                    liftState = LiftState.THREE_CUBE_HEIGHT;
                    break;
                default:
                    break;
            }
        }

        if (gamepadToUse.dpad_down && !dPadWasDown) {
            switch (liftState) {
                case HALF_CUBE_HEIGHT:
                    liftState = LiftState.ZERO_CUBE_HEIGHT;
                    break;
                case ONE_CUBE_HEIGHT:
                    liftState = LiftState.HALF_CUBE_HEIGHT;
                    break;
                case TWO_CUBE_HEIGHT:
                    liftState = LiftState.ONE_CUBE_HEIGHT;
                    break;
                case THREE_CUBE_HEIGHT:
                    liftState = LiftState.TWO_CUBE_HEIGHT;
                    break;
                default:
                    break;
            }
        }

        dPadWasUp = gamepadToUse.dpad_up;
        dPadWasDown = gamepadToUse.dpad_down;
    }

    public void manualControls(Gamepad gamepad) {
        if ((gamepad.right_trigger>0.1) || gamepad.left_trigger>0.1) {
            notManual = false;
            if (gamepad.right_trigger>0.01){
                liftPower = gamepad.right_trigger;
            }else if (gamepad.left_trigger>0.01) {
                liftPower = gamepad.left_trigger;
            }
            rightLiftMotor.setPower(liftPower);
        }
    }

    public void goToState(LiftState inLiftState) {
        switch (liftState) {
            case ZERO_CUBE_HEIGHT:
                goToTargetLiftPos(Karen.ZERO_CUBE_HEIGHT);
                break;
            case HALF_CUBE_HEIGHT:
                goToTargetLiftPos(Karen.HALF_CUBE_HEIGHT);
                break;
            case ONE_CUBE_HEIGHT:
                goToTargetLiftPos(Karen.ONE_CUBE_HEIGHT);
                break;
            case TWO_CUBE_HEIGHT:
                goToTargetLiftPos(Karen.TWO_CUBE_HEIGHT);
                break;
            case THREE_CUBE_HEIGHT:
                goToTargetLiftPos(Karen.THREE_CUBE_HEIGHT);
                break;
        }
    }

    public DcMotor getRightLiftMotor() {
        return rightLiftMotor;
    }

    public void setRightLiftMotor(DcMotor rightLiftMotor) {
        this.rightLiftMotor = rightLiftMotor;
    }

    public int getLiftPosition() {
        return rightLiftMotor.getCurrentPosition();
    }
}

package org.firstinspires.ftc.teamcode.UltronAutomodes;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.KarenOpMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;
import org.firstinspires.ftc.teamcode.KarenRobot.KarenAutoRobot;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.DriveSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.SensorSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.TeamMarkerSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.VisionSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.VuforiaSystem;
import org.firstinspires.ftc.teamcode.KarenUtil.SimpleColor;


/**
 * Created by Julian on 11/15/2017.
 */

public abstract class KarenAuto extends AutonomousProgram {
    public SensorSystem sensorSystem;
    public DriveSystem driveSystem;
    public LiftSystem liftSystem;
    public TeamMarkerSystem teamMarkerSystem;
    public VuforiaSystem vuforiaSystem;
    public VisionSystem visionSystem;

    protected double currentYaw;

    protected RelicRecoveryVuMark vuMark;

    private SimpleColor alliance;

    public KarenAuto(SimpleColor alliance) {this.alliance = alliance;}

    @Override
    protected Robot buildRobot() {
        KarenAutoRobot karen = new KarenAutoRobot(this, alliance, false);
        sensorSystem = (SensorSystem)karen.getSubSystem("sensor");
        driveSystem = (DriveSystem)karen.getSubSystem("drive");
        liftSystem = (LiftSystem)karen.getSubSystem("lift");
        teamMarkerSystem = (TeamMarkerSystem)karen.getSubSystem("marker");
        vuforiaSystem = (VuforiaSystem)karen.getSubSystem("vuforia");
        visionSystem = (VisionSystem)karen.getSubSystem("vision");
        return karen;
    }

    @Override
    public void postInit() {
        vuforiaSystem.activateVuforia();
    }

    @Override
    public void postMain() {
    }

    /**
     * turns to the left specified amount
     * @param target
     * @return
     */
    public double turn(double target) {
        // turns left some number of radians
        sensorSystem.updateGyro();
        currentYaw = sensorSystem.getYaw();

        double absoluteTarget = translateRadianAddition(currentYaw+target);//Converts properly? made by Julian, not the internet so may not be reliable
        return turnAbsolute(absoluteTarget);
    }

    public double turnAbsolute(double target) {
        // turns left some degrees
        double tolerance = 3*Math.PI/180;//3 degree
        double turnSpeed;
        sensorSystem.updateGyro();
        currentYaw = sensorSystem.getYaw();  //Set variables to gyro readings
        while (Math.abs(currentYaw - target) > tolerance && opModeIsActive()) {//Continue while the robot direction is further than three degrees from the target
            double actualDifference = (currentYaw-target);
            telemetry.addData("Target", target);
            telemetry.addData("Current", currentYaw);
            telemetry.addData("Difference", (currentYaw - target));
            telemetry.addData("Tolerance", Math.toRadians(tolerance));
            telemetry.addData("Actual Difference", actualDifference);
            telemetry.addData("ConversionFactor", sensorSystem.getConversionFactor());
            turnSpeed = Karen.MIN_TURN_SPEED;



            if (actualDifference > Math.PI) {
                driveSystem.setPower(-turnSpeed, turnSpeed);
                telemetry.addData("Turning", "left");
            }else {
                driveSystem.setPower(turnSpeed, -turnSpeed);
                telemetry.addData("Turning", "right");
            }

//            if (currentYaw > target) {  //if gyro is positive, we will turn right
//                driveSystem.setPower(-turnSpeed, turnSpeed);
//                telemetry.addData("Turning", "Right");
//            }
//
//            if (currentYaw < target) {  //if gyro is negative, we will turn left
//                driveSystem.setPower(turnSpeed, -turnSpeed);
//                telemetry.addData("Turning", "Left");
//            }
            sensorSystem.updateGyro();
            currentYaw = sensorSystem.getYaw();  //Set variables to gyro readings
            telemetry.update();
        }
        driveSystem.stopMotors();
        telemetry.update();
        return currentYaw;
    }

    public static double translateRadianAddition(double numIn) {
        double num = numIn;
        if (numIn < -Math.PI) {
            num = 2*Math.PI + numIn % (Math.PI * 2);
            return translateRadianAddition(num);
        } else if (numIn > Math.PI) {
            num = -2*Math.PI + numIn % (Math.PI * 2);
            return translateRadianAddition(num);
        } else {
            return num;
        }
    }

    public void driveTime(double speed, double inTime) {
        driveSystem.modeVoltage();
        long initialTime = System.currentTimeMillis();
        long stopTime = Math.round(inTime*1000 + initialTime);
        while ((System.currentTimeMillis() < stopTime) && opModeIsActive()) {
            telemetry.addData("Current Time: ", System.currentTimeMillis());
            telemetry.addData("Stop Time: ", stopTime);
            driveSystem.setPower(speed);
        }
        driveSystem.stop();
    }

    public void autoGoToLiftPos (LiftSystem.LiftState inLiftState, double power) {
        int targetPos = 0;
        int THRESHOLD = 100;

        switch (inLiftState) {
            case UP:
                targetPos = Karen.UP_HEIGHT;
                break;
            case DOWN:
                targetPos = Karen.DOWN_HEIGHT;
                break;
        }

        liftSystem.getLiftMotorUp().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && liftSystem.getLiftMotorUp().isBusy()) {
            liftSystem.goToTargetLiftPos(targetPos, power);
        }
        // ToDo: why is this doubled?
        while (opModeIsActive() && Math.abs(liftSystem.getLiftPos() - targetPos) > THRESHOLD) {
            liftSystem.goToTargetLiftPos(targetPos, power);
        }

        liftSystem.stop();

    }

    public void autoTurnDegreesRight(int targetDeg) {
        // 3 sec max to turn
        long startTime = System.currentTimeMillis();
        int targetPos = targetDeg*525/90;
        int rightCurrentPos = driveSystem.getRightEncoderValue();
        int leftCurrentPos = driveSystem.getLeftEncoderValue();
        int rightTargetPos = rightCurrentPos - targetPos;
        int leftTargetPos = leftCurrentPos + targetPos;
        driveSystem.modePosition();
        driveSystem.getDcMotors()[0].setTargetPosition(rightTargetPos);
        driveSystem.getDcMotors()[1].setTargetPosition(leftTargetPos);
        driveSystem.setPower(Karen.MIN_TURN_SPEED);

        while ((driveSystem.getDcMotors()[0].isBusy() || driveSystem.getDcMotors()[1].isBusy()) && opModeIsActive() && !(System.currentTimeMillis() > startTime + 3000)) {
            if (!driveSystem.getDcMotors()[0].isBusy()) {
                driveSystem.getDcMotors()[0].setPower(0);
            }
            if (!driveSystem.getDcMotors()[1].isBusy()) {
                driveSystem.getDcMotors()[1].setPower(0);
            }
            idle();
        }

        // makes sure that the motors are stopped no matter what
        driveSystem.stopMotors();
        driveSystem.modeVoltage();
    }

    public void autoDriveWithEncoders(int targetTicks) {
        driveSystem.modeReset();
        driveSystem.modePosition();
        // resets the encoder distance, so now we will go to the target spot and no additional math
        // is needed
        long startTime = System.currentTimeMillis();
        int rightCurrentPos = driveSystem.getRightEncoderValue();
        int leftCurrentPos = driveSystem.getLeftEncoderValue();
        driveSystem.getDcMotors()[0].setTargetPosition(targetTicks);
        driveSystem.getDcMotors()[1].setTargetPosition(targetTicks);
        driveSystem.setPower(Karen.MIN_TURN_SPEED);

        while ((driveSystem.getDcMotors()[0].isBusy() || driveSystem.getDcMotors()[1].isBusy()) && opModeIsActive() && !(System.currentTimeMillis() > startTime + 3000)) {
            if (!driveSystem.getDcMotors()[0].isBusy()) {
                driveSystem.getDcMotors()[0].setPower(0);
            }
            if (!driveSystem.getDcMotors()[1].isBusy()) {
                driveSystem.getDcMotors()[1].setPower(0);
            }
            idle();
        }

        // makes sure the motors are stopped no matter what
        driveSystem.stopMotors();
        driveSystem.modeVoltage();

    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.KarenOpMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;
import org.firstinspires.ftc.teamcode.KarenRobot.KarenAutoRobot;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.CubeSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.DriveSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.JewelArmSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.JewelColorSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.SensorSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.VuforiaSystem;
import org.firstinspires.ftc.teamcode.KarenUtil.SimpleColor;


/**
 * Created by Julian on 11/15/2017.
 */

public abstract class KarenAuto extends AutonomousProgram {
    public DriveSystem driveSystem;
    public LiftSystem liftSystem;
    public CubeSystem cubeSystem;
    public JewelArmSystem jewelArmSystem;
    public JewelColorSystem jewelColorSystem;
    public SensorSystem sensorSystem;
    public VuforiaSystem vuforiaSystem;
//    public RelicSystem relicSystem;

    protected double currentYaw;

    protected RelicRecoveryVuMark vuMark;

    private SimpleColor alliance;

    public KarenAuto(SimpleColor alliance) {this.alliance = alliance;}

    @Override
    protected Robot buildRobot() {
        KarenAutoRobot karen = new KarenAutoRobot(this, alliance, false);
        driveSystem = (DriveSystem)karen.getSubSystem("drive");
        liftSystem = (LiftSystem)karen.getSubSystem("lift");
        cubeSystem = (CubeSystem)karen.getSubSystem("cube");
        jewelArmSystem = (JewelArmSystem)karen.getSubSystem("jewelArm");
        sensorSystem = (SensorSystem)karen.getSubSystem("sensor");
        vuforiaSystem = (VuforiaSystem)karen.getSubSystem("vuforia");
//        relicSystem = (RelicSystem)karen.getSubSystem("relic");
        jewelColorSystem = (JewelColorSystem)karen.getSubSystem("jewelColor");
        return karen;
    }

    @Override
    public void postInit() {
        vuforiaSystem.activateVuforia();
    }

    @Override
    public void postMain() {
        autoGoToLiftPos(LiftSystem.LiftState.ZERO_CUBE_HEIGHT);
    }

    /**
     * turns to the left specified amount
     * @param target
     * @return
     */
    public double turn(double target) {
        sensorSystem.updateGyro();
        currentYaw = sensorSystem.getYaw();

        double absoluteTarget = translateRadianAddition(currentYaw+target);//Converts properly? made by Julian, not the internet so may not be reliable
        return turnAbsolute(absoluteTarget);
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

    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public double turnAbsolute(double target) {
        double tolerance = 4;//2 degrees
        double turnSpeed;
        sensorSystem.updateGyro();
        currentYaw = sensorSystem.getYaw();  //Set variables to gyro readings
        while (Math.abs(currentYaw - target) > tolerance && opModeIsActive()) {//Continue while the robot direction is further than three degrees from the target
            telemetry.addData("Target", Math.toDegrees(target));
            telemetry.addData("Current", Math.toDegrees(currentYaw));
            telemetry.addData("Difference", Math.toDegrees(currentYaw - target));

            turnSpeed = Karen.MIN_TURN_SPEED;

            if (currentYaw > target) {  //if gyro is positive, we will turn right
                driveSystem.driveAngle(0, Math.PI/2, turnSpeed);
                telemetry.addData("Turning", "Right");
            }

            if (currentYaw < target) {  //if gyro is negative, we will turn left
                driveSystem.driveAngle(0, Math.PI/2, -turnSpeed);
                telemetry.addData("Turning", "Left");
            }
            sensorSystem.updateGyro();
            currentYaw = sensorSystem.getYaw();  //Set variables to gyro readings
            telemetry.update();
        }
        telemetry.update();
        driveSystem.stopMotors();
        return currentYaw;
    }

    public void driveStraightForward(int distance, double power) {

        driveSystem.resetEncoders();
        double leftSpeed; //Power to feed the motors
        double rightSpeed;

        double target = sensorSystem.getYaw();  //Starting direction
        double startPosition = driveSystem.getRightEncoderValue();  //Starting position

        while (driveSystem.getRightEncoderValue() < distance + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            sensorSystem.updateGyro();
            currentYaw = sensorSystem.getYaw();  //Current direction

            leftSpeed = power + Math.toDegrees(currentYaw - target) / 100;  //Calculate speed for each side
            rightSpeed = power - Math.toDegrees(currentYaw - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            driveSystem.setTurnPower(rightSpeed,leftSpeed);

            telemetry.addData("Distance to go", distance + startPosition - sensorSystem.getYaw());
            telemetry.update();
        }

        driveSystem.stopMotors();
    }

    /**
     * Takes negative values
     * @param distance
     * @param power
     */
    public void driveStraightBackwards (int distance, double power) {

        driveSystem.resetEncoders();
        double leftSpeed; //Power to feed the motors
        double rightSpeed;

        double target = sensorSystem.getYaw();  //Starting direction
        double startPosition = driveSystem.getRightEncoderValue();  //Starting position

        while (driveSystem.getRightEncoderValue() > distance + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            sensorSystem.updateGyro();
            currentYaw = sensorSystem.getYaw();  //Current direction

            leftSpeed = power - Math.toDegrees(currentYaw - target) / 100;  //Calculate speed for each side
            rightSpeed = power + Math.toDegrees(currentYaw - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            driveSystem.setTurnPower(rightSpeed,leftSpeed);

            telemetry.addData("Distance to go", distance + startPosition - sensorSystem.getYaw());
            telemetry.update();
        }

        driveSystem.stopMotors();
    }

    public void driveForwardsToGivenPosition(double inPower, int distance) {
        driveSystem.resetEncoders();
        int currentRightPos = driveSystem.getRightEncoderValue();
        int currentLeftPos = driveSystem.getLeftEncoderValue();
        while ((currentRightPos<distance || currentLeftPos<distance) && opModeIsActive()) {
            if (currentLeftPos<distance) {
                driveSystem.getFrontLeft().setPower(inPower);
                driveSystem.getRearLeft().setPower(inPower);
            } else {
                driveSystem.getFrontLeft().setPower(0);
                driveSystem.getRearLeft().setPower(0);
            }
            if (currentRightPos<distance) {
                driveSystem.getFrontRight().setPower(inPower);
                driveSystem.getRearRight().setPower(inPower);
            } else {
                driveSystem.getFrontRight().setPower(0);
                driveSystem.getRearRight().setPower(0);
            }
            currentRightPos = driveSystem.getRightEncoderValue();
            currentLeftPos = driveSystem.getLeftEncoderValue();
            telemetry.addData("Right Pos", currentRightPos);
            telemetry.addData("Left Pos", currentLeftPos);
            telemetry.update();
        }
        driveSystem.stopMotors();
    }

    /**
     * takes negative values
     * @param inPower
     * @param distance
     */
    public void driveBackwardsToGivenPosition(double inPower, int distance) {
        driveSystem.resetEncoders();
        int currentRightPos = driveSystem.getRightEncoderValue();
        int currentLeftPos = driveSystem.getLeftEncoderValue();
        while ((currentRightPos>distance || currentLeftPos>distance) && opModeIsActive()) {
            if (currentLeftPos>distance) {
                driveSystem.getFrontLeft().setPower(inPower);
                driveSystem.getRearLeft().setPower(inPower);
            } else {
                driveSystem.getFrontLeft().setPower(0);
                driveSystem.getRearLeft().setPower(0);
            }
            if (currentRightPos>distance) {
                driveSystem.getFrontRight().setPower(inPower);
                driveSystem.getRearRight().setPower(inPower);
            } else {
                driveSystem.getFrontRight().setPower(0);
                driveSystem.getRearRight().setPower(0);
            }
            currentRightPos = driveSystem.getRightEncoderValue();
            currentLeftPos = driveSystem.getLeftEncoderValue();
            telemetry.addData("Right Pos", currentRightPos);
            telemetry.addData("Left Pos", currentLeftPos);
            telemetry.update();
        }
        driveSystem.stopMotors();
    }

    public void driveTime(double speed, double inTime) {
        long initialTime = System.currentTimeMillis();
        long stopTime = Math.round(inTime*1000 + initialTime);
        while ((System.currentTimeMillis() < stopTime) && opModeIsActive()) {
            driveSystem.driveForward(speed);
        }
    }

    public void autoGoToLiftPos(LiftSystem.LiftState inLiftState) {
        int targetPos = 0;
        int THRESHOLD = 100;

        switch (inLiftState) {
            case ZERO_CUBE_HEIGHT:
                targetPos = Karen.ZERO_CUBE_HEIGHT;
                break;
            case HALF_CUBE_HEIGHT:
                targetPos = Karen.HALF_CUBE_HEIGHT;
                break;
            case ONE_CUBE_HEIGHT:
                targetPos = Karen.ONE_CUBE_HEIGHT;
                break;
            case TWO_CUBE_HEIGHT:
                targetPos = Karen.TWO_CUBE_HEIGHT;
                break;
            case THREE_CUBE_HEIGHT:
                targetPos = Karen.THREE_CUBE_HEIGHT;
                break;
        }

        liftSystem.getRightLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && liftSystem.getRightLiftMotor().isBusy()) {
            liftSystem.goToTargetLiftPos(targetPos);
        }
        while (opModeIsActive() && Math.abs(liftSystem.getLiftPosition() - targetPos) > THRESHOLD) {
            liftSystem.goToTargetLiftPos(targetPos);
        }

        liftSystem.stop();

    }

    public SimpleColor identifyJewelColor(double time) {
        long startTime = System.currentTimeMillis();
        int sawRed = 0;
        int sawBlue = 0;
        while (opModeIsActive() && (System.currentTimeMillis() < (startTime + time))) {
            if (jewelColorSystem.getRedColor() > jewelColorSystem.getBlueColor()) {
                sawRed++;
            } else if (jewelColorSystem.getBlueColor() > jewelColorSystem.getRedColor()) {
                sawBlue++;
            }
        }

        if (sawBlue > sawRed) {
            return SimpleColor.BLUE;
        } else if (sawRed > sawBlue) {
            return SimpleColor.RED;
        } else {
            return SimpleColor.UNKNOWN;
        }

    }

    public VuforiaSystem.CryptoboxKey identifyVuMark(double time) {
        long stopVuSearch = System.currentTimeMillis() + 5000;
        VuforiaSystem.CryptoboxKey cryptoboxKey;
        int[] positionSeen = new int[3];//Right is 0, center is 1, left is 2
        while (opModeIsActive() && System.currentTimeMillis()< stopVuSearch) {
            vuMark = vuforiaSystem.checkForVuMark();
            switch (vuMark) {
                case RIGHT:
                    positionSeen[0]++;
                    break;
                case CENTER:
                    positionSeen[1]++;
                    break;
                case LEFT:
                    positionSeen[2]++;
                    break;
                case UNKNOWN:
                    break;
            }
        }

        int maxPos = 0;
        for (int pos:positionSeen) {
            if (pos>maxPos)
                maxPos = pos;
        }

        if (maxPos == positionSeen[0]) {
            cryptoboxKey = VuforiaSystem.CryptoboxKey.RIGHT;
        } else if (maxPos == positionSeen[1]) {
            cryptoboxKey = VuforiaSystem.CryptoboxKey.CENTER;
        } else {
            cryptoboxKey = VuforiaSystem.CryptoboxKey.LEFT;
        }
        return cryptoboxKey;
    }

    public RelicRecoveryVuMark oldIdentifyVuMark(double time) {
        int sawRight = 0;
        int sawCenter = 0;
        int sawLeft = 0;
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() < (startTime + time))) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(vuforiaSystem.relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                switch (vuMark) {
                    case RIGHT:
                        sawRight++;
                        break;
                    case CENTER:
                        sawCenter++;
                        break;
                    case LEFT:
                        sawLeft++;
                        break;
                }

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)vuforiaSystem.relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
        if (sawRight>sawCenter && sawRight>sawLeft) {
            return RelicRecoveryVuMark.RIGHT;
        } else if (sawCenter>sawRight && sawCenter>sawLeft) {
            return RelicRecoveryVuMark.CENTER;
        } else {
            return RelicRecoveryVuMark.LEFT;
        }
    }



    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}

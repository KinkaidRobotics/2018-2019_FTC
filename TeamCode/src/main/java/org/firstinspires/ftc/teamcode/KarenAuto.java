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
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.DriveSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.VuforiaSystem;
import org.firstinspires.ftc.teamcode.KarenUtil.SimpleColor;


/**
 * Created by Julian on 11/15/2017.
 */

public abstract class KarenAuto extends AutonomousProgram {
    public DriveSystem driveSystem;
    public LiftSystem liftSystem;
    public VuforiaSystem vuforiaSystem;

    protected double currentYaw;

    protected RelicRecoveryVuMark vuMark;

    private SimpleColor alliance;

    public KarenAuto(SimpleColor alliance) {this.alliance = alliance;}

    @Override
    protected Robot buildRobot() {
        KarenAutoRobot karen = new KarenAutoRobot(this, alliance, false);
        driveSystem = (DriveSystem)karen.getSubSystem("drive");
        liftSystem = (LiftSystem)karen.getSubSystem("lift");
        vuforiaSystem = (VuforiaSystem)karen.getSubSystem("vuforia");
        return karen;
    }

    @Override
    public void postInit() {
        vuforiaSystem.activateVuforia();
    }

    @Override
    public void postMain() {
    }

//    /**
//     * turns to the left specified amount
//     * @param target
//     * @return
//     */
//    public double turn(double target) {
//        sensorSystem.updateGyro();
//        currentYaw = sensorSystem.getYaw();
//
//        double absoluteTarget = translateRadianAddition(currentYaw+target);//Converts properly? made by Julian, not the internet so may not be reliable
//        return turnAbsolute(absoluteTarget);
//    }

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
        long initialTime = System.currentTimeMillis();
        long stopTime = Math.round(inTime*1000 + initialTime);
        while ((System.currentTimeMillis() < stopTime) && opModeIsActive()) {
            driveSystem.setPower(speed);
        }
    }

//    public VuforiaSystem.CryptoboxKey identifyVuMark(double time) {
//        long stopVuSearch = System.currentTimeMillis() + 5000;
//        VuforiaSystem.CryptoboxKey cryptoboxKey;
//        int[] positionSeen = new int[3];//Right is 0, center is 1, left is 2
//        while (opModeIsActive() && System.currentTimeMillis()< stopVuSearch) {
//            vuMark = vuforiaSystem.checkForVuMark();
//            switch (vuMark) {
//                case RIGHT:
//                    positionSeen[0]++;
//                    break;
//                case CENTER:
//                    positionSeen[1]++;
//                    break;
//                case LEFT:
//                    positionSeen[2]++;
//                    break;
//                case UNKNOWN:
//                    break;
//            }
//        }
//
//        int maxPos = 0;
//        for (int pos:positionSeen) {
//            if (pos>maxPos)
//                maxPos = pos;
//        }
//
//        if (maxPos == positionSeen[0]) {
//            cryptoboxKey = VuforiaSystem.CryptoboxKey.RIGHT;
//        } else if (maxPos == positionSeen[1]) {
//            cryptoboxKey = VuforiaSystem.CryptoboxKey.CENTER;
//        } else {
//            cryptoboxKey = VuforiaSystem.CryptoboxKey.LEFT;
//        }
//        return cryptoboxKey;
//    }

//    public RelicRecoveryVuMark oldIdentifyVuMark(double time) {
//        int sawRight = 0;
//        int sawCenter = 0;
//        int sawLeft = 0;
//        long startTime = System.currentTimeMillis();
//        while (opModeIsActive() && (System.currentTimeMillis() < (startTime + time))) {
//
//            /**
//             * See if any of the instances of {@link relicTemplate} are currently visible.
//             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
//             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
//             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
//             */
//            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(vuforiaSystem.relicTemplate);
//            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//                /* Found an instance of the template. In the actual game, you will probably
//                 * loop until this condition occurs, then move on to act accordingly depending
//                 * on which VuMark was visible. */
//                telemetry.addData("VuMark", "%s visible", vuMark);
//                switch (vuMark) {
//                    case RIGHT:
//                        sawRight++;
//                        break;
//                    case CENTER:
//                        sawCenter++;
//                        break;
//                    case LEFT:
//                        sawLeft++;
//                        break;
//                }
//
//                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
//                 * it is perhaps unlikely that you will actually need to act on this pose information, but
//                 * we illustrate it nevertheless, for completeness. */
//                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)vuforiaSystem.relicTemplate.getListener()).getPose();
//                telemetry.addData("Pose", format(pose));
//
//                /* We further illustrate how to decompose the pose into useful rotational and
//                 * translational components */
//                if (pose != null) {
//                    VectorF trans = pose.getTranslation();
//                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//
//                    // Extract the rotational components of the target relative to the robot
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;
//                }
//            }
//            else {
//                telemetry.addData("VuMark", "not visible");
//            }
//
//            telemetry.update();
//        }
//        if (sawRight>sawCenter && sawRight>sawLeft) {
//            return RelicRecoveryVuMark.RIGHT;
//        } else if (sawCenter>sawRight && sawCenter>sawLeft) {
//            return RelicRecoveryVuMark.CENTER;
//        } else {
//            return RelicRecoveryVuMark.LEFT;
//        }
//    }



    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}

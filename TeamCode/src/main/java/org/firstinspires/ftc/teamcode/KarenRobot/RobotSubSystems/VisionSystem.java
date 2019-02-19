package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

/**
 * Created by Julian on 11/9/2018
 */

public class VisionSystem extends SubSystem {
    // ranges from 1 to 638
    public double leftBound = 213; // need to calibrate these
    public double rightBound = 425;

    public enum GoldPos{
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }

    private GoldAlignDetector detector;

    public VisionSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        detector = new GoldAlignDetector();
        detector.init(hardwareMap().appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
        // the method to use is detector.getXPosition
    }

    @Override
    public void handle() {
        telemetry().addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
        telemetry().addData("X Pos" , detector.getXPosition()); // Gold X pos.
    }

    @Override
    public void stop() {
        detector.disable();
    }

    public GoldAlignDetector getDetector() {
        return detector;
    }

    public GoldPos getGoldPos(){
        if (detector.isFound()) {
            double xPos = detector.getXPosition();
            if (xPos < leftBound) {
                return GoldPos.LEFT;
            } else if (xPos > rightBound) {
                return GoldPos.RIGHT;
            } else {
                return GoldPos.CENTER;
            }
        } else {
            return GoldPos.NOT_FOUND;
        }
    }

    public boolean isCentered() {
        double xPos = detector.getXPosition();
        if (xPos > leftBound && xPos < rightBound)
            return true;
        return false;
    }
}

package org.firstinspires.ftc.teamcode.UltronRobot.RobotSubSystems;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;

import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.Robot;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.SubSystem;

/**
 * Created by Seb on 2/1/18.
 */

public class DogeCVSystem extends SubSystem {

    private JewelDetector jewelDetector = null;

    public DogeCVSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {

        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap().appContext, CameraViewDisplay.getInstance());
        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewelDetector.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        jewelDetector.disable();
    }

    public JewelDetector.JewelOrder getCurrentOrder() {
        return jewelDetector.getCurrentOrder();
    }

    public void enable() {
        jewelDetector.enable();
    }
}

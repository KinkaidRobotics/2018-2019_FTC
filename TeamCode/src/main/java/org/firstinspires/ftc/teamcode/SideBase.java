package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.JewelDetector;

import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.RobotSubSystems.LiftSystem;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.RobotSubSystems.VuforiaSystem;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronUtil.SimpleColor;

/**
 * Created by Julian on 2/2/2018.
 */

public class SideBase extends UltronAuto {

    public SideBase(SimpleColor alliance) {
        super(alliance);
    }

    @Override
    public void main() {
        VuforiaSystem.CryptoboxKey cryptoboxKey = identifyVuMark(3000);
        vuforiaSystem.stop();
        dogeCVSystem.enable();
        JewelDetector.JewelOrder jewelOrder = identifyJewelOrderVision(2000);
        if (getRobot().ALLIANCE == SimpleColor.RED) {
            redJewelKnock(jewelOrder);
            redVuMarkPositioning(cryptoboxKey);
        } else if (getRobot().ALLIANCE == SimpleColor.BLUE) {
            blueJewelKnock(jewelOrder);
            blueVuMarkPositioning(cryptoboxKey);
        }
        placeCube();
    }

    private void redJewelKnock(JewelDetector.JewelOrder jewelOrderIn) {
        switch (jewelOrderIn) {
            case BLUE_RED:
                driveBackwardsToGivenPosition(-0.5, -150);// go backwards
                jewelArmSystem.rightUp();
                waitFor(0.5);
                driveForwardsToGivenPosition(0.5, 2150);
                break;
            case RED_BLUE:
                driveForwardsToGivenPosition(0.5, 150);// go forwards
                jewelArmSystem.rightUp();//raise arm
                waitFor(0.5);
                driveForwardsToGivenPosition(0.5, 1800);
                break;
            case UNKNOWN:
                jewelArmSystem.rightUp();
                waitFor(0.5);
                driveForwardsToGivenPosition(0.5, 1900);
                break;
        }
    }

    private void blueJewelKnock(JewelDetector.JewelOrder jewelOrderIn) {
        switch (jewelOrderIn) {
            case BLUE_RED:
                driveForwardsToGivenPosition(0.5, 150);// go forwards
                jewelArmSystem.rightUp();//raise arm
                waitFor(0.5);
                driveForwardsToGivenPosition(0.5, 1800);
                break;
            case RED_BLUE:
                driveBackwardsToGivenPosition(-0.5, -150);// go backwards
                jewelArmSystem.rightUp();
                waitFor(0.5);
                driveForwardsToGivenPosition(0.5, 2150);
                break;
            case UNKNOWN:
                jewelArmSystem.rightUp();
                waitFor(0.5);
                driveForwardsToGivenPosition(0.5, 1900);
        }
    }

    private void redVuMarkPositioning(VuforiaSystem.CryptoboxKey cryptoboxKeyIn) {
        turnAbsolute(Math.PI/2);//Turn left 90 degrees
        switch (cryptoboxKeyIn) {
            case LEFT:
                driveForwardsToGivenPosition(0.5, 550);//Will likely need to be changed
            case CENTER:
                driveForwardsToGivenPosition(0.5, 350);//Will likely need to be changed
            case RIGHT:
                driveForwardsToGivenPosition(0.5, 250);//Will possibly need to be changed
        }
    }

    private void blueVuMarkPositioning(VuforiaSystem.CryptoboxKey cryptoboxKeyIn) {
        turnAbsolute(-Math.PI/2);//Turn left 90 degrees
        switch (cryptoboxKeyIn) {
            case LEFT:
                driveForwardsToGivenPosition(0.5, 250);//Will likely need to be changed
            case CENTER:
                driveForwardsToGivenPosition(0.5, 350);//Will likely need to be changed
            case RIGHT:
                driveForwardsToGivenPosition(0.5, 550);//Will possibly need to be changed
        }
    }

    private void placeCube() {
        turnAbsolute(0);//Counter turn
        autoGoToLiftPos(LiftSystem.LiftState.HALF_CUBE_HEIGHT);
        waitFor(0.5);
        driveTime(1,1);//Full speed into the cryptobox in case we are not lined up
        cubeSystem.openTop();
        driveBackwardsToGivenPosition(-0.5,-400);
        autoGoToLiftPos(LiftSystem.LiftState.ZERO_CUBE_HEIGHT);
        driveTime(1,1);
        waitFor(1);
        driveBackwardsToGivenPosition(-0.5,-300);
    }

}

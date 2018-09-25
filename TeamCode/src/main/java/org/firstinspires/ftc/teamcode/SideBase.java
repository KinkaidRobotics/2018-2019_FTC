package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.VuforiaSystem;
import org.firstinspires.ftc.teamcode.KarenUtil.SimpleColor;

/**
 * Created by Julian on 2/2/2018.
 */

public class SideBase extends KarenAuto {

    public SideBase(SimpleColor alliance) {
        super(alliance);
    }

    @Override
    public void main() {
        VuforiaSystem.CryptoboxKey cryptoboxKey = identifyVuMark(3000);
        vuforiaSystem.stop();
        placeCube();
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

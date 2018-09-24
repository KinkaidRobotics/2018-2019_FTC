package org.firstinspires.ftc.teamcode.UltronRobot.RobotSubSystems;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.Robot;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.Ultron;

/**
 * Created by Julian on 12/15/2017.
 */

public class VuforiaSystem extends SubSystem {

    public ClosableVuforiaLocalizer vuforia;
    public int cameraMonitorViewId;
    public VuforiaLocalizer.Parameters parameters;
    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;

    public enum CryptoboxKey{
        RIGHT,CENTER,LEFT
    }

    public VuforiaSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        cameraMonitorViewId = hardwareMap().appContext.getResources().getIdentifier(Ultron.cameraViewID, "id", hardwareMap().appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = Ultron.vuforiaLicenseKey;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new ClosableVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset(Ultron.vuMarkAsset);
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName(Ultron.relicTemplate); // can help in debugging; otherwise not necessary

        telemetry().addData(">", "Press Play to start");
        telemetry().update();
    }

    @Override
    public void handle() {
    }

    @Override
    public void stop() {
        vuforia.close();
    }

    public void activateVuforia() {
        relicTrackables.activate();
    }

    public RelicRecoveryVuMark checkForVuMark() {return RelicRecoveryVuMark.from(relicTemplate);}
}

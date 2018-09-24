package org.firstinspires.ftc.teamcode.UltronUtil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronAuto;
import org.firstinspires.ftc.teamcode.Kinkaid_Stuff.v3.UltronRobot.RobotSubSystems.VuforiaSystem;

/**
 * Created by Julian on 12/16/2017.
 */
@Autonomous(name = "VuMarkTest", group = "Util")
public class VuMarkTest extends UltronAuto {
    public VuMarkTest() {
        super(null);
    }

    @Override
    public void main() {
        VuforiaSystem.CryptoboxKey cryptoboxKey;
        RelicRecoveryVuMark vuMark = oldIdentifyVuMark(5000);

        switch (vuMark) {
            case RIGHT:
                telemetry.addData("I saw", "Right");
                cryptoboxKey = VuforiaSystem.CryptoboxKey.RIGHT;
                break;
            case CENTER:
                telemetry.addData("I saw", "Center");
                cryptoboxKey = VuforiaSystem.CryptoboxKey.CENTER;
                break;
            case LEFT:
                telemetry.addData("I saw", "Left");
                cryptoboxKey = VuforiaSystem.CryptoboxKey.LEFT;
        }

        telemetry.update();
        waitFor(5);
    }
}

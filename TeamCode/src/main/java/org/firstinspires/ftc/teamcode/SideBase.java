package org.firstinspires.ftc.teamcode;
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
        vuforiaSystem.stop();
    }
}

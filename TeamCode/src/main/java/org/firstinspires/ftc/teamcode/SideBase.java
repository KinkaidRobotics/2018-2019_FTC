package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.KarenUtil.SimpleColor;
import org.firstinspires.ftc.teamcode.UltronAutomodes.KarenAuto;

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

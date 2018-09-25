package org.firstinspires.ftc.teamcode.KarenRobot.General;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.KarenUtil.SimpleColor;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by Julian on 11/14/2017.
 */

public class Robot {
    private final Map<String, SubSystem> subSystems;
    private final Map<Class, SubSystem> classSubSystemMap;
    public final OpMode opMode;

    public volatile Gamepad gamepad1;

    public volatile Gamepad gamepad2;

    public final Telemetry telemetry;

    public final HardwareMap hardwareMap;

    public SimpleColor ALLIANCE;

    public boolean TWO_DRIVERS;

    public Robot(OpMode opMode) {
        this.opMode = opMode;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        subSystems = new HashMap<>();
        classSubSystemMap = new HashMap<>();
    }

    protected void putSubSystem(String name, SubSystem subSystem) {
        subSystems.put(name, subSystem);
        classSubSystemMap.put(subSystem.getClass(), subSystem);
    }

    public final void init() {
        for (SubSystem subSystem:subSystems.values()) {
            try {
                subSystem.init();
                telemetry.addData("Initialized", "");
                telemetry.update();
            } catch(Exception ex) {
                telemetry.addData("Error!!!", ex.getMessage());
                telemetry.addData("subsystem", subSystem.getClass());
                ex.printStackTrace();
            }
        }
        telemetry.addData("Finished", "Init");
        telemetry.update();
    }

    public final void driverControlledUpdate() {
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        for (SubSystem subSystem:subSystems.values()) {
            try {
                subSystem.handle();
            } catch(Exception ex) {
                telemetry.addData("Error!!!", ex.getMessage());
                ex.printStackTrace();
            }
        }
    }

    public final void stopAllComponents() {
        for(SubSystem subSystem:subSystems.values()) {
            try {
                subSystem.stop();
            } catch(Exception ex) {
                telemetry.addData("Error!!!", ex.getMessage());
                ex.printStackTrace();
            }
        }
        //Threading.clearStopAllThreads(); this was written in the KNO3 code, but I don't known what it does
    }

    /**
     * Call this to get a subsystem of the robot.
     * @param name The name of the subsystem to fetch.
     * @return The subsystem registered in the robot. null if it does not exist.
     */
    public final SubSystem getSubSystem(String name) {
        return subSystems.get(name);
    }

    public final <T extends SubSystem> T getSubSystem(Class<T> tClass) {
        return (T) classSubSystemMap.get(tClass);
    }

    /**
     * Replace a subsystem that was already registered. Or add a new one.
     * @param name Name of the subsystem.
     * @param subSystem Subsystem to override with/add
     * @return Returns the subsystem passed as a parameter. This is so that you can use it in a builder style.
     */
    public final SubSystem eOverrideSubSystem(String name, SubSystem subSystem) {
        subSystems.put(name, subSystem);
        classSubSystemMap.put(subSystem.getClass(), subSystem);
        return subSystem;
    }

}

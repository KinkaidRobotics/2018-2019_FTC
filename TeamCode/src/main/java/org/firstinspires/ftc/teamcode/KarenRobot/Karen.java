package org.firstinspires.ftc.teamcode.KarenRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.DriveSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.LiftSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems.SensorSystem;
import org.firstinspires.ftc.teamcode.KarenUtil.SimpleColor;

/**
 * Created by Julian on 11/14/2017.
 */

public class Karen extends Robot {

    //DriveSystem
    public static final String DRIVE_FL_KEY = "LFD";//DC Port 0
    public static final String DRIVE_RL_KEY = "LRD";//DC Port 1
    public static final String DRIVE_FR_KEY = "RFD";//DC Port 2
    public static final String DRIVE_RR_KEY = "RRD";//DC Port 3
    public static final double MIN_TURN_SPEED = 0.35;

    //LiftSystem
    public static final String LIFT_R_WINCH_KEY = "RL";//MR Port 0
    public static final int ZERO_CUBE_HEIGHT = 0;
    public static final int HALF_CUBE_HEIGHT = 2480;
    public static final int ONE_CUBE_HEIGHT = 3800;
    public static final int TWO_CUBE_HEIGHT = 0;
    public static final int THREE_CUBE_HEIGHT = 0;

    //IntakeSystem
    public static final String RIGHT_TOP_SERVO_KEY = "RTS";//Servo Port 0
    public static final String LEFT_TOP_SERVO_KEY = "LTS";//Servo Port 1
//    public static final String RIGHT_LOWER_SERVO_KEY = "RLS";
//    public static final String LEFT_LOWER_SERVO_KEY = "LLS";
    public static final double RIGHT_TOP_SERVO_OPEN = 0.41;
    public static final double RIGHT_TOP_SERVO_RELEASED = 0.2;
    public static final double RIGHT_TOP_SERVO_CLOSED = 0;
    public static final double LEFT_TOP_SERVO_OPEN = 0;
    public static final double LEFT_TOP_SERVO_RELEASED = 0.2;
    public static final double LEFT_TOP_SERVO_CLOSED = 0.55;
//    public static final double RIGHT_LOWER_SERVO_OPEN = 0;
//    public static final double RIGHT_LOWER_SERVO_CLOSED = 0;
//    public static final double LEFT_LOWER_SERVO_OPEN = 0;
//    public static final double LEFT_LOWER_SERVO_CLOSED = 0;

    //JewelArmSystem
    public static final String RIGHT_TOP_JEWEL_SERVO_KEY = "RUJ";//Servo Port 2
    public static final String RIGHT_LOWER_JEWEL_SERVO_KEY = "RLJ";//Servo Port 3
//    public static final String LEFT_TOP_JEWEL_SERVO_KEY = "LUJ";//Servo Port 4
//    public static final String LEFT_LOWER_JEWEL_SERVO_KEY = "LLJ";//Servo Port 5
    public static final double RIGHT_TOP_SERVO_DOWN = 0;//1
    public static final double RIGHT_LOWER_SERVO_DOWN = 0.2;//0.34
    public static final double RIGHT_TOP_SERVO_UP = 0.45;//0.45
    public static final double RIGHT_LOWER_SERVO_UP = 1;//0
//    public static final double LEFT_TOP_SERVO_DOWN = 0;
//    public static final double LEFT_LOWER_SERVO_DOWN = 0;
//    public static final double LEFT_TOP_SERVO_UP = 0;
//    public static final double LEFT_LOWER_SERVO_UP = 0;

    //RelicSystem
    public static final String CLAW_SERVO_KEY = "CLAW";
    public static final String RELIC_ROTATE_SERVO_KEY = "RRotate";
    public static final String RELIC_EXTENSION_KEY = "RExtension";
    public static final double RELIC_CLAW_OPEN = 0.5;
    public static final double RELIC_CLAW_CLOSE = 0;
    public static final double RELIC_UP = 0.13;
    public static final double RELIC_DOWN = 0.07;
    public static final int RELIC_EXTENSION_OUT = 3000;//Value not calibrated
    public static final int RELIC_EXTENSION_IN = 0;

    //SensorSystem
    public static final String IMU_KEY = "IMU";//I2C 0
    public static final String COLOR_SENSOR_KEY = "CS";//I2C 1

    //VuforiaSystem
    public static final String vuforiaLicenseKey = "Ac8vX/3/////AAAAGRLM+gjmWUoAhO8Kns/kDEpxkCW1u1lX1uZW3r/rphUKMxf6jDE4oTHTZ3F/J0GwvD+hMipwaFBziRnmUDZHHXNM//wCa80uaLKJPlK7KkLqmz8dedEKTeMZomxY0T/dAee/7nGRrOtTihtJZvQJNv9RHvgWGF8pR0/lzZobtkU7V3uv+DC/gXY9sJxn/yQdxjxBXuW83wzVcT8tsefn7G8+9T9Til2ZOt/SNV8ilLdfiYFfMUaDdbJnTmaQlhITHP2dtmu71op2u5tsHlABrhs1RDwq3DPC+X+DLJfaPV8kTrgxgo0yxGjDgDmprty6x/BZkv7GD347DQlGBPRuhoSCzrtrR9XtC+FYeuVsbWOu";
    public static final String cameraViewID = "cameraMonitorViewId";
    public static final String relicTemplate = "relicVuMarkTemplate";
    public static final String vuMarkAsset = "RelicVuMark";

    public Karen(OpMode opMode, SimpleColor alliance, Boolean twoDrivers) {
        super(opMode);
        ALLIANCE = alliance;
        TWO_DRIVERS = twoDrivers;
        putSubSystem("sensor", new SensorSystem(this));
        putSubSystem("drive", new DriveSystem(this));
        putSubSystem("lift", new LiftSystem(this));
        putSubSystem("intake", new IntakeSystem(this));
    }
}

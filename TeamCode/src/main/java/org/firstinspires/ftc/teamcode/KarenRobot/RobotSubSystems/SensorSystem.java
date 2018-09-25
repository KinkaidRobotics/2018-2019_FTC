package org.firstinspires.ftc.teamcode.KarenRobot.RobotSubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.KarenRobot.General.Robot;
import org.firstinspires.ftc.teamcode.KarenRobot.General.SubSystem;
import org.firstinspires.ftc.teamcode.KarenRobot.Karen;

/**
 * Created by Julian on 11/25/2017.
 */

public class SensorSystem extends SubSystem {

    private BNO055IMU imu;
    private Orientation angles;
    private double yaw;
    private double roll;
    private double pitch;
    private double conversionFactor;

    public SensorSystem(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap().get(BNO055IMU.class, Karen.IMU_KEY);
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    }

    @Override
    public void handle() {
        updateGyro();

        displayValues();
    }

    @Override
    public void stop() {

    }

    public void updateGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yaw = (angles.firstAngle + conversionFactor);
        roll = (angles.secondAngle + Math.PI*2)%(2*Math.PI);
        pitch = (angles.thirdAngle + Math.PI*2)%(2*Math.PI);
    }

    public double getYaw() {
        return -yaw;
    }

    public double getRoll() {
        return roll;
    }

    public double getPitch() {
        return pitch;
    }

    public void displayValues() {
        telemetry().addData("Yaw", yaw);
        telemetry().addData("Roll", roll);
        telemetry().addData("Pitch", pitch);
    }

    public void resetGyro() {
        updateGyro();
        conversionFactor = getYaw();
    }
}

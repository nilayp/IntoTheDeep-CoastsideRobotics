package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="TestSensors", group="Robot")
public class TestSensors extends LinearOpMode {

    Servo andymarkServo0;
    Servo revServo1;
    ColorSensor colorSensor;
    int fromRightRedPercent;
    int frontRightBluePercent;
    int frontRightGreenPercent;
    int frontRightAlpha;
    IMU imu;
    DistanceSensor frontDistanceSensor;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    @Override
    public void runOpMode() {
        andymarkServo0 = hardwareMap.servo.get("servo0");
        revServo1 = hardwareMap.servo.get("servo1");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        imu = hardwareMap.get(IMU.class, "imu");
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
           getColorPercentages();
           updateTelemetry();
        }
    }

    private void getColorPercentages() {
        int fromRightRed = colorSensor.red();
        int frontRightBlue = colorSensor.blue();
        int frontRightGreen = colorSensor.green();
        frontRightAlpha = colorSensor.alpha();

        int total = fromRightRed + frontRightBlue + frontRightGreen;
        fromRightRedPercent = (fromRightRed * 100) / total;
        frontRightBluePercent = (frontRightBlue * 100) / total;
        frontRightGreenPercent = (frontRightGreen * 100) / total;
    }

    public void updateTelemetry() {
        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
        telemetry.addData("Front Right Red", colorSensor.red());
        telemetry.addData("Front Right Blue", colorSensor.blue());
        telemetry.addData("Front Right Green", colorSensor.green());
        telemetry.addData("Front Right Alpha", colorSensor.alpha());

        telemetry.addData("Front Right Red %", fromRightRedPercent);
        telemetry.addData("Front Right Blue %", frontRightBluePercent);
        telemetry.addData("Front Right Green %", frontRightGreenPercent);
        telemetry.addData("Front Right Alpha", frontRightAlpha);

        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);

        telemetry.addData("Distance from object in inches", frontDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

}
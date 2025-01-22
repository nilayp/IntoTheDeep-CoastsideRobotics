package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="DriveToObservationZone", group="Robot")
public class DriveToObservationZone extends LinearOpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor liftArmMotor;
    Servo clawServo;
    Servo extendArmServo;

    boolean distanceSensorTriggered = false;
    DistanceSensor frontDistanceSensor;

    IMU imu;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    @Override
    public void runOpMode() {
        backLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("rearRightDrive");
        liftArmMotor = hardwareMap.dcMotor.get("liftArmMotor");
        extendArmServo = hardwareMap.servo.get("extendArmServo");
        clawServo = hardwareMap.servo.get("claw");
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
        imu = hardwareMap.get(IMU.class, "imu");

        // reverses the left, lift and extend arm motors because they are mounted backwards
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        liftArmMotor.setDirection(DcMotor.Direction.REVERSE);

        liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // set the initial positions. The claw is closed and the arm is tucked in.

        clawServo.setPosition(1.0);
        extendArmServo.setPosition(1.0);

        // getColorPercentages();
        updateTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            // The code inside the loop needs to do the following actions:
            // - Drive forward until the distance sensor detects an object within 13 inches

            if (!distanceSensorTriggered){
                // Move forward until we get close enough to the wall.
                driveForwardUntilNearWall(0.4);
            }
            updateTelemetry();
        }
    }

    private void driveForwardUntilNearWall(double power) {
        drive(power);
        // Stop if the front distance sensor detects an object within 13 inches
        if (frontDistanceSensor.getDistance(DistanceUnit.INCH) < 13) {
            drive(0.0);
            distanceSensorTriggered = true;
        }
    }

    public void drive(double power) {
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void updateTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);

        telemetry.addData("Distance from object in inches", frontDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("distanceSensorTriggeredStatus", distanceSensorTriggered);

        telemetry.update();
    }
}

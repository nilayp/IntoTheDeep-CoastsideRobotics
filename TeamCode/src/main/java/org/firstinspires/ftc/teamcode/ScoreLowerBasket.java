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

@Autonomous(name="ScoreLowerBasket", group="Robot")
public class ScoreLowerBasket extends LinearOpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor liftArmMotor;
    Servo clawServo;
    Servo extendArmServo;

    boolean armRaised = false;
    boolean departedWall = false;
    boolean turnedLeft = false;
    boolean distanceSensorTriggered = false;
    boolean rotateAtBasket = false;
    boolean extendArmOut = false;
    boolean clawRetracted = false;
    boolean extendArmIn = false;
    boolean armDropped = false;
    boolean backupAfterBasket = false;
    DistanceSensor frontDistanceSensor;

    IMU imu;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    int liftArmPositionTuckedIn = 0;
    int liftArmPositionScoringBottomBasket = 551;

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

        updateTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            // The code inside the loop needs to do the following actions:
            // - Raise the arm to the scoring position. The sample will be in the claw.
            // - depart the wall
            // - turn left 90 degrees
            // - drive forward until you are close to the wall
            // - rotate to 105 degrees of the starting position (~30 degrees from current position)
            // - extend the arm out
            // - retract the claw
            // - backup to avoid touching the basket upon retracting the claw
            // - retract the arm
            // - lower the arm to the starting position
            // Each of these actions sets a boolean value true so they aren't
            // repeated.

            if (!armRaised) {
                moveArmToPosition(liftArmPositionScoringBottomBasket, "armRaised", true);
            } else if (!departedWall) {
                driveWithDuration(0.5, 500, "departedWall", true);
            } else if (!turnedLeft) {
                leftTurn(0.3);
            } else if (!distanceSensorTriggered){
                // Move forward until we get close enough to the wall.
                driveForwardUntilNearWall(0.4);
            } else if (!rotateAtBasket) {
                rotate(105, 0.3, "rotateAtBasket", true);
            } else if (!extendArmOut) {
                toggleExtendArm(0.6, "extendArmOut", true, 1000);
            } else if (!clawRetracted) {
                toggleClawServo(0.0, "clawRetracted", true);
            } else if (!backupAfterBasket) {
                driveWithDuration(-0.5, 500, "backupAfterBasket", true);
            } else if (!extendArmIn) {
                toggleExtendArm(1.0, "extendArmIn", true, 1000);
            } else if (!armDropped) {
                moveArmToPosition(liftArmPositionTuckedIn, "armDropped", true);
            }
            updateTelemetry();
        }
    }

    private void toggleClawServo(double position, String fieldName, boolean value) {
        clawServo.setPosition(position);
        sleep(2000);
        switch (fieldName) {
            case "clawRetracted":
                clawRetracted = value;
                break;
        }
    }

    private void driveForwardUntilNearWall(double power) {
        drive(power);
        // Stop if the front distance sensor detects an object within 16 inches
        if (frontDistanceSensor.getDistance(DistanceUnit.INCH) < 16) {
            drive(0.0);
            distanceSensorTriggered = true;
        }
    }

    public void drive(double power) {
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void driveWithDuration(double power, int duration, String fieldName, boolean value) {
        drive(power);
        sleep(duration);
        switch (fieldName) {
            case "departedWall":
                departedWall = value;
                break;
            case "backupAfterBasket":
                backupAfterBasket = value;
                break;
        }
        drive(0.0);
    }

    public void rotate(int degree, double power, String fieldName, boolean value) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        if (orientation.getYaw(AngleUnit.DEGREES) < degree) {
            backLeftDrive.setPower(-power);
            backRightDrive.setPower(power);
        } else if (orientation.getYaw(AngleUnit.DEGREES) >= degree) {
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            switch (fieldName) {
                case "rotateAtBasket":
                    rotateAtBasket = value;
                    break;
            }
        }
    }
    public void leftTurn(double power) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        if (orientation.getYaw(AngleUnit.DEGREES) < 90) {
            backLeftDrive.setPower(-power);
            backRightDrive.setPower(power);
        } else if (orientation.getYaw(AngleUnit.DEGREES) >= 90) {
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            turnedLeft = true;
        }
    }
    // Method to move arm to a specific encoder position
    public void moveArmToPosition(int targetPosition, String fieldName, boolean value) {
        liftArmMotor.setTargetPosition(targetPosition);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftArmMotor.setPower(0.5);

        // Wait until the arm reaches the position
        while (opModeIsActive() && liftArmMotor.isBusy()) {
            updateTelemetry();
        }
        switch (fieldName) {
            case "armDropped":
                armDropped = value;
                break;
            case "armRaised":
                armRaised = value;
                break;
        }
    }

    private void toggleExtendArm(double position, String fieldName, boolean value, int delay) {
        extendArmServo.setPosition(position);
        sleep(delay);
        switch (fieldName) {
            case "extendArmOut":
                extendArmOut = value;
                break;
            case "extendArmIn":
                extendArmIn = value;
                break;
        }
    }

    public void updateTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);

        telemetry.addData("Distance from object in inches", frontDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Lift Arm Position", liftArmMotor.getCurrentPosition());
        telemetry.addData("Claw Position", clawServo.getPosition());
        telemetry.addData("Extend Arm Position", extendArmServo.getPosition());
        telemetry.addData("armRaisedStatus", armRaised);
        telemetry.addData("departedWallStatus", departedWall);
        telemetry.addData("turnedLeftStatus", turnedLeft);
        telemetry.addData("distanceSensorTriggeredStatus", distanceSensorTriggered);
        telemetry.addData("rotateAtBasketStatus", rotateAtBasket);
        telemetry.addData("extendArmOutStatus", extendArmOut);
        telemetry.addData("clawRetractedStatus", clawRetracted);
        telemetry.addData("backupAfterBasketStatus", backupAfterBasket);
        telemetry.addData("extendArmInStatus", extendArmIn);
        telemetry.addData("armDroppedStatus", armDropped);

        telemetry.update();
    }
}

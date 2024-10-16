package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Simple TeleOp Tank", group="Robot")
public class SimpleTeleOpTank extends OpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    double driveSpeed;
    double leftPower;
    double rightPower;

    public void init() {
        backLeftDrive = hardwareMap.dcMotor.get("back_left_motor");
        backRightDrive = hardwareMap.dcMotor.get("back_right_motor");

        // reverses the left motor because it is mounted upside down
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // starts the robot at full speed
        driveSpeed = 1.0;

    }
    public void loop() {
        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;

        // toggles the drive speed between 0.5 and 1.0
        // by pressing the right and left bumper
        if (gamepad1.left_bumper) {
            driveSpeed = 0.5;
        }
        if (gamepad1.right_bumper) {
            driveSpeed = 1.0;
        }

        tankDrive(leftPower, rightPower, driveSpeed);
        updateTelemetry();
    }
    public void tankDrive(double leftPower, double rightPower, double driveSpeed) {
        backLeftDrive.setPower(leftPower * driveSpeed);
        backRightDrive.setPower(rightPower * driveSpeed);
    }
    public void updateTelemetry() {
        telemetry.addData("GamePad1 left_stick_y", leftPower);
        telemetry.addData("GamePad1 right_stick_y", rightPower);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Left Power", backLeftDrive.getPower());
        telemetry.addData("Right Power", backRightDrive.getPower());
        telemetry.update();
    }
}

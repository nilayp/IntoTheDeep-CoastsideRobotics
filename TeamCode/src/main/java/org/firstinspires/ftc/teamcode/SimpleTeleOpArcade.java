package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Simple TeleOp Arcade", group="Robot")
public class SimpleTeleOpArcade extends OpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    double driveSpeed;
    double leftPower;
    double turnPower;

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
        turnPower = gamepad1.right_stick_x;

        // toggles the drive speed between 0.5 and 1.0
        // by pressing the right and left bumper
        if (gamepad1.left_bumper) {
            driveSpeed = 0.5;
        }
        if (gamepad1.right_bumper) {
            driveSpeed = 1.0;
        }

        arcadeDrive(leftPower, turnPower, driveSpeed);
        updateTelemetry();
    }
    public void arcadeDrive(double forwardPower, double turnPower, double driveSpeed) {
        double leftPower = forwardPower + turnPower;
        double rightPower = forwardPower - turnPower;

        // scales the power of the motors if the maximum power is greater than 1.0
        // by taking the maximum of the absolute values of the left and right powers

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower = leftPower / max;
            rightPower = rightPower / max;
        }
        backLeftDrive.setPower(leftPower * driveSpeed);
        backRightDrive.setPower(rightPower * driveSpeed);
    }
    public void updateTelemetry() {
        telemetry.addData("GamePad1 left_stick_y", leftPower);
        telemetry.addData("GamePad1 right_stick_x", turnPower);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Left Power", backLeftDrive.getPower());
        telemetry.addData("Right Power", backRightDrive.getPower());
        telemetry.update();
    }

}

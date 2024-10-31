package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Simple TeleOp Arcade", group="Robot")
public class SimpleTeleOpArcade extends OpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    Servo clawServo;

    double driveSpeed;
    double leftPower;
    double turnPower;

    public void init() {
        backLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("rearRightDrive");
        clawServo = hardwareMap.servo.get("claw");

        // reverses the left motor because it is mounted upside down
       backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // starts the robot at full speed
        driveSpeed = 0.75;
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
            driveSpeed = 0.75;
        }

        arcadeDrive(leftPower, turnPower, driveSpeed);
        clawServo(gamepad2.right_trigger);
        updateTelemetry();
    }

    private void clawServo(double position) {
        clawServo.setPosition(position);
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
        telemetry.addData("GamePad1 left_stick_x", turnPower);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Left Power (Motor)", backLeftDrive.getPower());
        telemetry.addData("Right Power (Motor)", backRightDrive.getPower());
        telemetry.addData("Claw Position (Servo)", clawServo.getPosition());
        telemetry.update();
    }

}

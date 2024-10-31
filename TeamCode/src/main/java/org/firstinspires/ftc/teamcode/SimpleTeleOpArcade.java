package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Simple TeleOp Arcade", group="Robot")
public class SimpleTeleOpArcade extends OpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor liftArmMotor;

    // Specify positions for the lift arm
    int liftArmPositionTuckedIn = 0;
    int liftArmPositionScoringTopBasket = 360;
    int liftArmPositionClimbLowerRung = 563;
    Servo clawServo;

    double driveSpeed;
    double leftPower;
    double turnPower;

    public void init() {
        backLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("rearRightDrive");
        liftArmMotor = hardwareMap.dcMotor.get("liftArmMotor");
        clawServo = hardwareMap.servo.get("claw");

        // reverses the left & lift arm motors because they are mounted backwards
       backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
       liftArmMotor.setDirection(DcMotor.Direction.REVERSE);

       liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       liftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // starts the robot at 0.75 speed
        driveSpeed = 0.75;
    }
    public void loop() {
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

        if (gamepad2.dpad_up){
            liftArmToPosition(liftArmPositionScoringTopBasket);
        } else if (gamepad2.dpad_down) {
            liftArmToPosition(liftArmPositionTuckedIn);
        } else if (gamepad2.dpad_right) {
            liftArmIncrement(true);
        } else if (gamepad2.dpad_left) {
            liftArmIncrement(false);
        }

        updateTelemetry();
    }

    private void liftArmToPosition(int position) {
        liftArmMotor.setTargetPosition(position);
        System.out.println("Lift arm to position: " + position);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftArmMotor.setPower(0.30);
    }
    private void liftArmIncrement(boolean isMovingUp) {
        // increments the lift arm motor by 10, but first checks if the arm should be moved
        // up or down based on the current position. We don't want the arm to move past a certain
        // point or the robot may tip over.

        int currentPosition = liftArmMotor.getCurrentPosition();
        int increment = 10;
        if (isMovingUp) {
            if ((currentPosition + increment) <= liftArmPositionClimbLowerRung) {
                liftArmToPosition(currentPosition + increment);
            }
        } else {
            if (currentPosition - increment >= liftArmPositionTuckedIn) {
                liftArmToPosition(currentPosition - increment);
            }
        }
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
        telemetry.addData("Lift Arm Position (Motor)", liftArmMotor.getCurrentPosition());
        telemetry.addData("Claw Position (Servo)", clawServo.getPosition());
        telemetry.update();
    }

}

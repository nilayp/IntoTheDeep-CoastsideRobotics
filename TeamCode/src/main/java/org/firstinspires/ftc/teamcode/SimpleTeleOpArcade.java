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

    DcMotor extendArmMotor;
    int extendArmPositionTuckedIn = 0;

    // The positions specified are in negative numbers because there is something
    // wrong with the encoder. It seems that the encoder is reversed from the motor. As
    // the FTC SDK doesn't have a way to reverse the encoder on it's own, we can't currently
    // use the RUN_TO_POSITION mode. We will need to use the RUN_USING_ENCODER mode and manually
    // controller it with the joystick. This is a temporary solution until we can figure out
    // how to reverse the encoder or fix whatever is wrong with the motor. Lastly, the encoder
    // values will be used to limit the range of motion to prevent damage to the extend arm mechanism.

    int extendArmPickTuckedIn = 0;
    int extendArmTopBasket = -1100;
    double armPower;
    double armPowerSpeed;

    double driveSpeed;
    double leftPower;
    double turnPower;

    public void init() {
        backLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("rearRightDrive");
        liftArmMotor = hardwareMap.dcMotor.get("liftArmMotor");
        extendArmMotor = hardwareMap.dcMotor.get("extendArmMotor");
        clawServo = hardwareMap.servo.get("claw");

        // reverses the left, lift and extend arm motors because they are mounted backwards
       backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
       liftArmMotor.setDirection(DcMotor.Direction.REVERSE);
       extendArmMotor.setDirection(DcMotor.Direction.REVERSE);

       liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       liftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       extendArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // starts the robot at 0.75 speed
        driveSpeed = 0.75;

        // starts the arm power at slow speed
        armPowerSpeed = 0.3;
    }
    public void loop() {
        leftPower = gamepad1.left_stick_y;
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

        // if the armPower is greater than 0, the arm will extend out. If the armPower is less than 0,
        // the arm will retract. If the armPower is 0, the arm will stop where it is.

        armPower = -gamepad2.left_stick_x;

        // throttles the arm power by pressing the left or right bumper

        if (gamepad2.left_bumper) {
            armPowerSpeed = 0.3;
        } else if (gamepad2.right_bumper) {
            armPowerSpeed = 0.6;
        }

        extendArm(armPower);
        updateTelemetry();
    }

    private void liftArmToPosition(int position) {
        liftArmMotor.setTargetPosition(position);
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
    private void extendArm(double power) {

        // Use the encoder values to limit the range of motion of the extend arm mechanism
        // to prevent damage to the robot.

        int currentPosition = extendArmMotor.getCurrentPosition();
        if ((power > 0 && currentPosition > extendArmTopBasket) ||
            (power < 0 && currentPosition < extendArmPickTuckedIn)) {
            extendArmMotor.setPower(power * armPowerSpeed);
        } else {
            extendArmMotor.setPower(0);
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
        telemetry.addData("Extend Arm Position (Motor)", extendArmMotor.getCurrentPosition());
        telemetry.addData("Extend Arm Power (Motor)", (armPower * armPowerSpeed));
        telemetry.addData("Extend Arm Power Speed", armPowerSpeed);
        telemetry.addData("Claw Position (Servo)", clawServo.getPosition());
        telemetry.update();
    }

}

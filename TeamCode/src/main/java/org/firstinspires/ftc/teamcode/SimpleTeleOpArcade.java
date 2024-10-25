package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Simple TeleOp Arcade", group="Robot")
public class SimpleTeleOpArcade extends OpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor liftArm;
    DcMotor climbArm;

    Servo clawServo;
    Servo extendArmServo;

    ColorSensor frontRightColorSensor;
    int fromRightRedPercent;
    int frontRightBluePercent;
    int frontRightGreenPercent;
    int frontRightAlpha;

    double driveSpeed;
    double leftPower;
    double turnPower;

    public void init() {
        backLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("rearRightDrive");
        climbArm = hardwareMap.dcMotor.get("climbArm");
        liftArm = hardwareMap.dcMotor.get("liftArm");
        clawServo = hardwareMap.servo.get("claw");
        extendArmServo = hardwareMap.servo.get("extendArm");
        frontRightColorSensor = hardwareMap.colorSensor.get("frontRightColorSensor");

        // reverses the left motor because it is mounted upside down
       backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // starts the robot at full speed
        driveSpeed = 1.0;

    }
    public void loop() {
        leftPower = -gamepad1.left_stick_y;
        turnPower = gamepad1.left_stick_x;

        // toggles the drive speed between 0.5 and 1.0
        // by pressing the right and left bumper
        if (gamepad1.left_bumper) {
            driveSpeed = 0.5;
        }
        if (gamepad1.right_bumper) {
            driveSpeed = 1.0;
        }

        arcadeDrive(leftPower, turnPower, driveSpeed);
        liftArm(gamepad2.right_stick_x);
        climbArm(gamepad2.left_stick_x);
        clawServo(gamepad2.right_trigger);
        extendArmServo(gamepad2.left_trigger);
        getColorPercentages();
        updateTelemetry();
    }
   private void getColorPercentages() {
        int fromRightRed = frontRightColorSensor.red();
        int frontRightBlue = frontRightColorSensor.blue();
        int frontRightGreen = frontRightColorSensor.green();
        frontRightAlpha = frontRightColorSensor.alpha();

        int total = fromRightRed + frontRightBlue + frontRightGreen;
        fromRightRedPercent = (fromRightRed * 100) / total;
        frontRightBluePercent = (frontRightBlue * 100) / total;
        frontRightGreenPercent = (frontRightGreen * 100) / total;
    }

    private void clawServo(double position) {
        clawServo.setPosition(position);
    }

    private void extendArmServo(double position) {
        extendArmServo.setPosition(position);
    }

    private void climbArm(double power) {
        climbArm.setPower(power);
    }

    public void liftArm(double power) {
        liftArm.setPower(power);
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
        telemetry.addData("Climb Arm Power (Motor)", climbArm.getPower());
        telemetry.addData("Lift Arm Power (Motor)", liftArm.getPower());
        telemetry.addData("Claw Position (Servo)", clawServo.getPosition());
        telemetry.addData("Extend Arm Position (Servo)", extendArmServo.getPosition());
        telemetry.addData("Front Right Red %", fromRightRedPercent);
        telemetry.addData("Front Right Blue %", frontRightBluePercent);
        telemetry.addData("Front Right Green %", frontRightGreenPercent);
        telemetry.addData("Front Right Alpha", frontRightAlpha);
        telemetry.update();
    }

}

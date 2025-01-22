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
    int liftArmPositionLowerPosition = 132;
    int liftArmPositionPickupUnderSub = 230;
    int liftArmPositionScoringBottomBasket = 551;
    int liftArmPositionScoringTopBasket = 845;
    int liftArmPositionMaxTall = 1500;
    int liftArmPositionClimbLowerRung = 1435;

    Servo clawServo;
    Servo extendArmServo;
    double extensionTarget = 1.0; //extension target position; also the position at startup
    double manualExtend = 0.005; //this is the manual extension factor bigger moves faster**ADJUST THIS NUMBER AS NEEDED!**

    double armPower;
    double armPowerSpeed;

    double driveSpeed;
    double leftPower;
    double turnPower;

    public void init() {
        backLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("rearRightDrive");
        liftArmMotor = hardwareMap.dcMotor.get("liftArmMotor");
        extendArmServo = hardwareMap.servo.get("extendArmServo");
        clawServo = hardwareMap.servo.get("claw");

        // reverses the left, lift and extend arm motors because they are mounted backwards
       backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
       liftArmMotor.setDirection(DcMotor.Direction.REVERSE);

       liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       liftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // retract the arm
       extendArmServo();

        // starts the robot at 0.75 speed
        driveSpeed = 0.75;

        // starts the arm power at slow speed
        armPowerSpeed = 0.3;

        // close the claw
        clawServo(1.0);
        updateTelemetry();

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
        extendArmServo();

        if (gamepad2.triangle) {
            liftArmToPosition(liftArmPositionScoringTopBasket, 0.30);
        }else if (gamepad2.circle) {
            liftArmToPosition(liftArmPositionScoringBottomBasket, 0.30);
        } else if (gamepad2.x) { // the square button on the PS4
            liftArmToPosition(liftArmPositionPickupUnderSub, 0.30);
        } else if (gamepad2.cross) { // the x button on the PS4
            liftArmToPosition(liftArmPositionLowerPosition, 0.30);
        } else if (gamepad2.dpad_down) {
            liftArmToPosition(liftArmPositionTuckedIn, 0.30);
        } else if (gamepad2.dpad_right) {
            liftArmIncrement(true);
        } else if (gamepad2.dpad_left) {
            liftArmIncrement(false);
        } else if (gamepad2.touchpad) {
            liftArmToPosition(liftArmPositionClimbLowerRung,0.3);
        } else if (gamepad2.right_bumper) {
            liftArmToPosition(liftArmPositionClimbLowerRung, 0.3);
        }

        // if the armPower is greater than 0, the arm will extend out. If the armPower is less than 0,
        // the arm will retract. If the armPower is 0, the arm will stop where it is.

        armPower = -gamepad2.right_stick_y;
        updateTelemetry();
    }

    private void liftArmToPosition(int position, double power) {
        liftArmMotor.setTargetPosition(position);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftArmMotor.setPower(power);
    }
    private void liftArmIncrement(boolean isMovingUp) {
        // increments the lift arm motor by 10, but first checks if the arm should be moved
        // up or down based on the current position. We don't want the arm to move past a certain
        // point or the robot may tip over.

        int currentPosition = liftArmMotor.getCurrentPosition();
        int increment = 10;
        if (isMovingUp) {
            if ((currentPosition + increment) <= liftArmPositionMaxTall) {
                liftArmToPosition(currentPosition + increment, 0.30);
            }
        } else {
            if (currentPosition - increment >= liftArmPositionTuckedIn) {
                liftArmToPosition(currentPosition - increment,0.30);
            }
        }
    }
    private void extendArmServo() {
        extendArmServo.setPosition(extensionTarget);
        if (gamepad2.left_bumper){ //manual extension in
            if (extensionTarget <= 1){
                extensionTarget = extensionTarget + manualExtend;
            }
            else if (extensionTarget > 1){
                extensionTarget = 1;
            }
        }
        else if (gamepad2.left_trigger > 0){ //manual extension out
            if (extensionTarget > 0){
                extensionTarget = extensionTarget - manualExtend;
            }
            else if (extensionTarget <0){
                extensionTarget = 0;
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
        telemetry.addData("Extend Arm Position (Servo)", extendArmServo.getPosition());
        telemetry.addData("Extend Arm Power (Motor)", (armPower * armPowerSpeed));
        telemetry.addData("Extend Arm Power Speed", armPowerSpeed);
        telemetry.addData("Claw Position (Servo)", clawServo.getPosition());
        telemetry.update();
    }
}

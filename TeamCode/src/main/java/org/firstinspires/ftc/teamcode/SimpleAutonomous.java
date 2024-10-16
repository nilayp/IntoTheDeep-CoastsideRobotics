package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous", group="Robot")
public class SimpleAutonomous extends LinearOpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    @Override
    public void runOpMode() {
        backLeftDrive = hardwareMap.dcMotor.get("back_left_motor");
        backRightDrive = hardwareMap.dcMotor.get("back_right_motor");

        // Set the direction of the motors. The left motor needs to be
        // reversed because it is mounted upside down.

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        tankDrive(.5, .5);
        sleep(1000);
        tankDrive(0, 0);

        while (opModeIsActive()) {
            // Loop code here
        }
    }
    public void tankDrive(double leftPower, double rightPower) {
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);

        telemetry.addData("Left Power", backLeftDrive.getPower());
        telemetry.addData("Right Power", backRightDrive.getPower());
        telemetry.update();
    }
}

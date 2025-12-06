package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HardwareDrive;

@Autonomous(name = "Auto Drive Forward 12in", group = "Auto")
public class AutoDriveForward12 extends LinearOpMode {

    HardwareDrive robot = new HardwareDrive();

    // ---- CHANGE THESE FOR YOUR ROBOT ----
    static final double COUNTS_PER_MOTOR_REV = 537.7;   // Example: GoBILDA 312 RPM
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No gearing
    static final double WHEEL_DIAMETER_INCHES = 3.78;   // Your wheel size (GoBILDA mecanum)
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    // --------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Reset encoders for all 4 motors
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        // ---- Move forward 12 inches ----
        encoderDrive(0.5, 12, 3.0); // power, inches, timeout
    }


    /**
     * Encoder drive method to move forward/backward precise distance.
     */
    public void encoderDrive(double speed, double inches, double timeoutS) {

        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;

        // Calculate target ticks
        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        newFLTarget = robot.frontLeft.getCurrentPosition() + moveCounts;
        newFRTarget = robot.frontRight.getCurrentPosition() + moveCounts;
        newBLTarget = robot.backLeft.getCurrentPosition() + moveCounts;
        newBRTarget = robot.backRight.getCurrentPosition() + moveCounts;

        // Set targets
        robot.frontLeft.setTargetPosition(newFLTarget);
        robot.frontRight.setTargetPosition(newFRTarget);
        robot.backLeft.setTargetPosition(newBLTarget);
        robot.backRight.setTargetPosition(newBRTarget);

        // Switch to RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motors
        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(speed);
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(speed);

        // Run until done or timeout
        double startTime = getRuntime();
        while (opModeIsActive() &&
                (getRuntime() - startTime < timeoutS) &&
                (robot.frontLeft.isBusy() && robot.frontRight.isBusy()
                        && robot.backLeft.isBusy() && robot.backRight.isBusy())) {

            telemetry.addData("Target", newFLTarget);
            telemetry.addData("Current FL", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Current FR", robot.frontRight.getCurrentPosition());
            telemetry.addData("Current FL", robot.backLeft.getCurrentPosition());
            telemetry.addData("Current FR", robot.backRight.getCurrentPosition());
            telemetry.addData("Counts Per Inch", COUNTS_PER_INCH);
            telemetry.update();
        }

        // Stop and reset mode
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


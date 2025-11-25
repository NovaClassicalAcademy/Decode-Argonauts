package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.HardwareDrive;

import java.util.List;

@Autonomous(name="DriveToAprilTag")
public class DriveToAprilTag extends LinearOpMode {

    HardwareDrive drive = new HardwareDrive();

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // ---------------------------
        // INIT HARDWARE
        // ---------------------------
        drive.init(hardwareMap);

        // ---------------------------
        // INIT APRILTAG
        // ---------------------------
        tagProcessor = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ====================================
        // FIND APRILTAG WITH ID = 20
        // ====================================
        AprilTagDetection tag = null;

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();
            tag = findTagWithID(detections, 20);

            if (tag != null) {
                telemetry.addLine("FOUND TAG 20!");
                telemetry.update();
                break;
            }

            telemetry.addLine("Searching for AprilTag ID 20...");
            telemetry.update();
        }

        if (tag == null) {
            stopMotors();
            telemetry.addLine("Tag 20 not found. Stopping.");
            telemetry.update();
            return;
        }

        // ====================================
        // DRIVE TOWARD TAG UNTIL 35 INCHES
        // ====================================
        driveTowardTag();
    }



    // -------------------------------------------
    // FIND TAG BY ID
    // -------------------------------------------
    private AprilTagDetection findTagWithID(List<AprilTagDetection> list, int id) {
        for (AprilTagDetection d : list) {
            if (d.id == id) return d;
        }
        return null;
    }



    // --------------------------------------------------
    // MAIN MOTION LOOP: Drive until 35 inches
    // --------------------------------------------------
    private void driveTowardTag() {

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = tagProcessor.getDetections();
            AprilTagDetection tag = findTagWithID(detections, 20);

            if (tag == null) {
                stopMotors();
                telemetry.addLine("Lost Tag 20!");
                telemetry.update();
                return;
            }

            double distance = tag.ftcPose.range;     // inches away
            double bearing  = tag.ftcPose.bearing;   // left/right angle
            double yaw      = tag.ftcPose.yaw;       // tag rotation

            telemetry.addData("Distance", distance);
            telemetry.addData("Bearing", bearing);
            telemetry.addData("Yaw", yaw);

            // Stop robot at EXACT distance
            if (distance <= 35) {
                stopMotors();
                telemetry.addLine("Reached 35 inches from Tag 20!");
                telemetry.update();
                return;
            }

            // -----------------------------
            // MOVEMENT (NO PID)
            // -----------------------------
            double forward = 0.18;     // slow forward
            double strafe  = 0.0;
            double turn    = 0.0;

            // Left/right correction
            if (bearing > 3)  strafe = 0.12;      // tag is to the right → move right
            if (bearing < -3) strafe = -0.12;     // tag is to the left  → move left

            // Turn correction
            if (yaw > 4)  turn = -0.10;           // rotate robot left
            if (yaw < -4) turn =  0.10;           // rotate robot right

            // Mecanum mixing
            double fl = forward + strafe + turn;
            double fr = forward - strafe - turn;
            double bl = forward - strafe + turn;
            double br = forward + strafe - turn;

            drive.frontLeft.setPower(-fl);
            drive.frontRight.setPower(-fr);
            drive.backLeft.setPower(-bl);

            drive.backRight.setPower(-br);

            telemetry.update();
        }

        stopMotors();
    }



    // ------------------------------------
    // STOP ALL DRIVE MOTORS
    // ------------------------------------
    private void stopMotors() {
        drive.frontLeft.setPower(0);
        drive.frontRight.setPower(0);
        drive.backLeft.setPower(0);
        drive.backRight.setPower(0);
    }
}


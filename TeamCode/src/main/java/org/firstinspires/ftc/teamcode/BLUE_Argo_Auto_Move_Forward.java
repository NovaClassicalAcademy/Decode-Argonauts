package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.HardwareDrive;
@Autonomous(name = "BLUE Move Forward Auto")
public class BLUE_Argo_Auto_Move_Forward extends LinearOpMode {

private HardwareDrive robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareDrive();
        robot.init(hardwareMap);

        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        // Move forward (power, time in milliseconds)
        moveForward(-0.1, 1500); // 0.5 power for 2 seconds

        stopMotors();
        telemetry.addLine("Done!");
        telemetry.update();
    }

    private void moveForward(double power, long timeMs) {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);

        sleep(timeMs);  // Robot moves during this time
    }

    private void stopMotors() {
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }
}

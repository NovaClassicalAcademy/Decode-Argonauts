package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Robot Centric Drive", group="Drive")
public class RobotCentricDrive extends LinearOpMode {

    public HardwareDrive robot = new HardwareDrive();
    //@Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Joystick inputs
            double y = -gamepad1.left_stick_y;  // Forward/Back
            double x = gamepad1.left_stick_x;   // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

            // Mecanum formula (Robot-Centric)
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Normalize powers to avoid exceeding 1.0
            double max = Math.max(1.0, Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backRightPower));

           //Move motor based on the power needed
            robot.moveMotor(robot.frontLeft, frontLeftPower / max,
                    robot.frontRight, frontRightPower / max,
                    robot.backLeft, backLeftPower / max,
                    robot.backRight, backRightPower / max
                    );



        }
    }
}

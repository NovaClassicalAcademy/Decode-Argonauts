package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Configuration.MAX_MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.Configuration.MIN_MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.Configuration.MOTOR_STOP;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_MOVE_TIME;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_PUSH_MAX;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_PUSH_MIN;
import static org.firstinspires.ftc.teamcode.Configuration.SLIDE_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Configuration.SLIDE_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.Configuration.SLIDE_TIMEOUT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Robot Centric Drive", group="Drive")
public class RobotCentricDrive extends LinearOpMode {

    public HardwareDrive robot = new HardwareDrive();
    //@Override
    private boolean aPressed = false;
    private boolean trayMoving = false;
     // Debounce flags
    private boolean xPressed = false;
    private boolean bPressed = false;

    private double drive_Speed;

    public void runOpMode() {
        robot.init(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper)

            {
                drive_Speed = MIN_MOTOR_SPEED;
            }
            else {
                drive_Speed = MAX_MOTOR_SPEED;
            }


            // Joystick inputs
            double y = -gamepad1.left_stick_y;  // Forward/Back
            double x = gamepad1.left_stick_x;   // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

            // Mecanum formula (Robot-Centric)
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            frontLeftPower = frontLeftPower * drive_Speed;
            frontRightPower = frontRightPower * drive_Speed;
            backLeftPower = backLeftPower * drive_Speed;
            backRightPower = backRightPower * drive_Speed;

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
//Intake
            if (gamepad1.right_trigger>0.05) { // intake motor brings balls in
                robot.intakeMotor.setPower(MAX_MOTOR_SPEED);
            }
            else
            {
                robot.intakeMotor.setPower(MOTOR_STOP);
            }
            if (gamepad1.left_trigger>0.05) { // push balls out, just in case
                robot.intakeMotor.setPower(-MAX_MOTOR_SPEED);
            }
            else
            {
                robot.intakeMotor.setPower(MOTOR_STOP);
            }

            //Outtake
            if (gamepad2.right_trigger>0.05) {
                robot.outtakeMotor.setPower(MAX_MOTOR_SPEED);
            }
            else
            {
                robot.outtakeMotor.setPower(MOTOR_STOP);
            }

/* Will use for 2nd comp
// Slides
            if (gamepad1.right_bumper) {
                robot.moveSlidesToPosition(SLIDE_MAX_HEIGHT, SLIDE_TIMEOUT);
            }
            if (gamepad1.left_bumper) {
                robot.moveSlidesToPosition(SLIDE_MIN_HEIGHT, SLIDE_TIMEOUT);
            }
*/
//SERVO MOVEMENT
// Next position (X)
            if (gamepad2.x && !xPressed) {
                xPressed = true;
                robot.nextTrayPosition();
            }
            if (!gamepad2.x) xPressed = false;

            // Previous position (B)
            if (gamepad2.b && !bPressed) {
                bPressed = true;
                robot.prevTrayPosition();
            }
            if (!gamepad2.b) bPressed = false;

            if (gamepad2.right_bumper) {
                robot.pushServo.setPosition(SERVO_PUSH_MAX);
                sleep(200);
                robot.pushServo.setPosition(SERVO_PUSH_MIN);
            }
            else if (gamepad2.left_bumper) {
                robot.pushServo.setPosition(SERVO_PUSH_MIN);
            }


        }
}
}
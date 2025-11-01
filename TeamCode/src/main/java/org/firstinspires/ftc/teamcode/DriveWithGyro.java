package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Configuration.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.FRONT_RIGHT_MOTOR;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//*TELEMETRY- Set power so that it will move but do sleep 2 sec so the servo moves*//

@TeleOp(name="Drive With Gyro-Field centric", group="Teleop")
//@Disabled
public class DriveWithGyro extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor sliderMotor = null;
    public double triggerSensitivityDeposit;
    public double triggerSensitivityIntake;
    private Servo servoMain;
    private CRServo servoGrab= null;
    //This is the Gyro (actually the Inertial Measurement Unit)
    private IMU imu;

    private double motorPower = 1;
    // Target positions for the slider (in encoder ticks)
    private int positionUp = 5800;   // Example target position for slider up
    private int positionDown = 0;     // Example target position for slider down
    private Servo Claw_Intake;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Drivetrain
        frontLeftMotor = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR);//Hub - Port #2
        backLeftMotor = hardwareMap.dcMotor.get(BACK_LEFT_MOTOR);//Hub - Port # 1
        frontRightMotor = hardwareMap.dcMotor.get(FRONT_RIGHT_MOTOR);//Hub - Port #0
        backRightMotor = hardwareMap.dcMotor.get(BACK_RIGHT_MOTOR);//Hub - Port #3

        // Define the IMU (gyro sensor)
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        //Field-centric initialization - end
        imu.resetYaw();  //reset the gyro

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     *
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //drvetrain
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double speedMax = 1;

        telemetry.addData("left Stick X: ", leftStickX);
        telemetry.addData("left Stick Y: ", leftStickY);
        telemetry.addData("Heading: ", botHeading);

        //*************************
        //* Field-centric driving *
        //*************************

        // Rotate the movement direction counter to the bot's rotation
        double rotX = leftStickX * Math.cos(-botHeading) - leftStickY * Math.sin(-botHeading);
        double rotY = leftStickX * Math.sin(-botHeading) + leftStickY * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);

        double frontLeftPower = ((rotY + rotX + rightStickX) / denominator) * speedMax;
        double backLeftPower = ((rotY - rotX + rightStickX) / denominator) * speedMax;
        double frontRightPower = ((rotY - rotX - rightStickX) / denominator) * speedMax;
        double backRightPower = ((rotY + rotX - rightStickX) / denominator) * speedMax;

        frontLeftMotor.setPower(-frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

     /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

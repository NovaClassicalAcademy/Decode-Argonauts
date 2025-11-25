package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Configuration.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.Configuration.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.MOTOR_STOP;
import static org.firstinspires.ftc.teamcode.Configuration.OUTTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_MOTOR_PUSH;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_MOTOR_TRAY;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_PUSH_MIN;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_TRAY_MIN;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_TRAY_SLOT1;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_TRAY_SLOT2;
import static org.firstinspires.ftc.teamcode.Configuration.SERVO_TRAY_SLOT3;
import static org.firstinspires.ftc.teamcode.Configuration.SLIDER_HOLD_SPEED;
//import static org.firstinspires.ftc.teamcode.Configuration.SLIDER_MOTOR_LEFT;
//import static org.firstinspires.ftc.teamcode.Configuration.SLIDER_MOTOR_RIGHT;
import static org.firstinspires.ftc.teamcode.Configuration.SLIDER_SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

//Declare the motors for the drivetrain,map the hardware and functions for the drivetrain
public class HardwareDrive {

    // Declare DrivetrainMotors
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    //Slider Motors
    public DcMotor leftSlider = null;
    public DcMotor rightSlider = null;

    //intake Motor
    public DcMotor intakeMotor = null;

    //outtake Motor
    public DcMotor outtakeMotor = null;
    //Servo motor
    public Servo trayServo = null;        // for standard positional servo
    // public CRServo trayServo = null;   // for continuous rotation servo

    public Servo pushServo = null;        // for standard positional servo
    // Preset tray positions (class-level variables)
    public final double[] TRAY_POSITIONS = {SERVO_TRAY_SLOT1, SERVO_TRAY_SLOT2, SERVO_TRAY_SLOT3};  // Low, Medium, High
    public int currentIndex = 0;  // Tracks current tray position

    // Initialize hardware from the configuration
    public void init(HardwareMap hwMap) {

        // Map motors from Driver Station config names
        //DRIVETRAIN
            frontLeft = hwMap.get(DcMotor.class,FRONT_LEFT_MOTOR);
            frontRight = hwMap.get(DcMotor.class, FRONT_RIGHT_MOTOR);
            backLeft = hwMap.get(DcMotor.class, BACK_LEFT_MOTOR);
            backRight = hwMap.get(DcMotor.class, BACK_RIGHT_MOTOR);
        //SLIDERS
        /* Will use for 2nd comp.
            leftSlider = hwMap.get(DcMotor.class, SLIDER_MOTOR_LEFT);
            rightSlider = hwMap.get(DcMotor.class, SLIDER_MOTOR_RIGHT);

         */
        //INTAKE
            intakeMotor = hwMap.get(DcMotor.class, INTAKE_MOTOR);
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        //OUTTAKE
            outtakeMotor = hwMap.get(DcMotor.class, OUTTAKE_MOTOR);
            outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set motor directions
        //DRIVETRAIN
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);
        //SLIDERS
           /* Will use for 2nd comp.
            leftSlider.setDirection(DcMotor.Direction.REVERSE);
            rightSlider.setDirection(DcMotor.Direction.FORWARD);
            */

        //Stop and reset encoders
        //DRIVETRAIN
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //SLIDERS
           /* Will ise for 2nd comp.
            leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           */
        // Optionally set zero power behavior
        //DRIVETRAIN
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       //SLIDERS
           /* Will use for 2nd comp.
            leftSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            */
       //Run using encoders
       //DRIVETRAIN
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       //SLIDERS
        /* Will use for 2nd comp.
            leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
       //Servo configuration
            trayServo = hwMap.get(Servo.class, SERVO_MOTOR_TRAY); // or CRServo.class if continuous
            initializeTrayServo(trayServo);

            pushServo = hwMap.get(Servo.class, SERVO_MOTOR_PUSH); // or CRServo.class if continuous
            initializePushServo(pushServo);

        // Set to initial position
        trayServo.setPosition(TRAY_POSITIONS[currentIndex]);
    }
    public void moveMotor(DcMotor frontLeft, double frontLeftPower,
                          DcMotor frontRight, double frontRightPower,
                          DcMotor backLeft, double backLeftPower,
                          DcMotor backRight, double backRightPower)
        {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }
    public void moveMotor(DcMotor frontLeft,
                          DcMotor frontRight,
                          DcMotor backLeft,
                          DcMotor backRight, double motorPower)
    {
        frontLeft.setPower(motorPower);
        frontRight.setPower(motorPower);
        backLeft.setPower(motorPower);
        backRight.setPower(motorPower);
    }

//Move the slider to a target position
    public void moveSlidesToPosition(int targetTicks, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();

        leftSlider.setTargetPosition(targetTicks);
        rightSlider.setTargetPosition(targetTicks);

        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlider.setPower(SLIDER_SPEED);
        rightSlider.setPower(SLIDER_SPEED);

        timer.reset();

        // Run both slides until done or timeout
        while ((leftSlider.isBusy() || rightSlider.isBusy()) && timer.seconds() < timeoutSeconds) {
            // Optional: you can add telemetry here if you want
        }

        // Stop motors
        leftSlider.setPower(MOTOR_STOP);
        rightSlider.setPower(MOTOR_STOP);

        // Switch back to RUN_USING_ENCODER to hold position gently
        leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply small hold power to keep the slides from drifting down
        leftSlider.setPower(SLIDER_HOLD_SPEED);
        rightSlider.setPower(SLIDER_HOLD_SPEED);
    }
//Servo Tray movements
    public void dumpTray() {
        trayServo.setPosition(0.8);
    }

    public void resetTray() {
        trayServo.setPosition(0.2);
    }
    public void initializeTrayServo(Servo trayServo)
    {
        // Initialize to starting position (only for standard servo)
        trayServo.setPosition(TRAY_POSITIONS[currentIndex]);
    }

    public void initializePushServo(Servo pushServo)
    {
        // Initialize to starting position (only for standard servo)
        pushServo.setPosition(SERVO_PUSH_MIN);
    }
    // Move Servo to the next preset position
    public void nextTrayPosition() {
        currentIndex++;
        if (currentIndex >= TRAY_POSITIONS.length) {
            currentIndex = TRAY_POSITIONS.length - 1; // stay at max
        }
        trayServo.setPosition(TRAY_POSITIONS[currentIndex]);
    }

    // Move Servo to the previous preset position
    public void prevTrayPosition() {
        currentIndex--;
        if (currentIndex < 0) {
            currentIndex = 0; // stay at min
        }
        trayServo.setPosition(TRAY_POSITIONS[currentIndex]);
    }

    // Optional: get current position for telemetry
    public double getCurrentPosition() {
        return TRAY_POSITIONS[currentIndex];
    }

    //Autonomous functions
    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int newLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(newLeftTarget);
        backLeft.setTargetPosition(newLeftTarget);
        frontRight.setTargetPosition(newRightTarget);
        backRight.setTargetPosition(newRightTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (frontLeft.isBusy() && frontRight.isBusy()) {
            // Loop until done
        }

        stopMotors();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void setBrakeMode() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}

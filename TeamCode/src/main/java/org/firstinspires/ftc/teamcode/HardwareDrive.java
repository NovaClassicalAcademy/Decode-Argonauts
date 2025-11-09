package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Configuration.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Configuration.FRONT_RIGHT_MOTOR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Declare the motors for the drivetrain,map the hardware and functions for the drivetrain
public class HardwareDrive {

    // Declare Motors
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    // Initialize hardware from the configuration
    public void init(HardwareMap hwMap) {

        // Map motors from Driver Station config names
        frontLeft = hwMap.get(DcMotor.class,FRONT_LEFT_MOTOR);
        frontRight = hwMap.get(DcMotor.class, FRONT_RIGHT_MOTOR);
        backLeft = hwMap.get(DcMotor.class, BACK_LEFT_MOTOR);
        backRight = hwMap.get(DcMotor.class, BACK_RIGHT_MOTOR);

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Optionally set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

}

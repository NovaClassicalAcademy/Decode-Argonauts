package org.firstinspires.ftc.teamcode;

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
        frontLeft = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hwMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hwMap.get(DcMotor.class, "backLeftMotor");
        backRight = hwMap.get(DcMotor.class, "backRightMotor");

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
    public void moveMotor(double frontLeftPower,
                          double frontRightPower,
                          double backLeftPower,
                          double backRightPower)
        {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }

}

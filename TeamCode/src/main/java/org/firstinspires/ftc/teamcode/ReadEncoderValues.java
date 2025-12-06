package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Read Encoder Values")
public class ReadEncoderValues extends LinearOpMode {

    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront"); // use your config name
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront"); // use your config name
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear"); // use your config name
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear"); // use your config name

        // Always reset encoders before reading predictable values
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {

            int position1 = frontLeft.getCurrentPosition();  // ticks
            double velocity1 = frontLeft.getVelocity();      // ticks/sec
            int position2 = backLeft.getCurrentPosition();  // ticks
            double velocity2 = backLeft.getVelocity();      // ticks/sec
            int position3 = frontRight.getCurrentPosition();  // ticks
            double velocity3 = frontRight.getVelocity();      // ticks/sec
            int position4 = backRight.getCurrentPosition();  // ticks
            double velocity4 = backRight.getVelocity();      // ticks/sec

            telemetry.addData("frontLeft Encoder Position", position1);
            telemetry.addData("frontLeft Encoder Velocity", velocity1);
            telemetry.addData("backLeft Encoder Position", position2);
            telemetry.addData("backLeft Encoder Velocity", velocity2);
            telemetry.addData("frontRight Encoder Position", position3);
            telemetry.addData("frontRight Encoder Velocity", velocity3);
            telemetry.addData("backRight Encoder Position", position4);
            telemetry.addData("backRight Encoder Velocity", velocity4);
            telemetry.update();
        }
    }
}

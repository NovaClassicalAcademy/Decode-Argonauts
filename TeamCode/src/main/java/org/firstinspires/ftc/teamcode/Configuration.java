package org.firstinspires.ftc.teamcode;

public class Configuration {

    // Motor ports - Drivetrain - REV Hub
    public static final int FRONT_LEFT_MOTOR_PORT = 3;
    public static final int BACK_LEFT_MOTOR_PORT = 2;
    public static final int FRONT_RIGHT_MOTOR_PORT = 1;
    public static final int BACK_RIGHT_MOTOR_PORT = 0;

    //intake Motor port
    public static final int INTAKE_MOTOR_PORT = 0;

    //Motor port - Servo
    public static final int SERVO_MOTOR_PORT = 0;

    //Motor Name - Servo
    public static final String SERVO_MOTOR_TRAY = "servoMotorTray";//3

    //Servo positions
    public static final double SERVO_TRAY_MIN = 0.2;
    public static final double SERVO_MOVE_TIME = 0.4;
    public static final double SERVO_TRAY_SLOT1 = 0.0;
    public static final double SERVO_TRAY_SLOT2 = 0.5;
    public static final double SERVO_TRAY_SLOT3 = 1.0;

    // Motor port - Intake

    // Motor Names - Drivetrain
    public static final String FRONT_LEFT_MOTOR = "frontLeftMotor";//3
    public static final String BACK_LEFT_MOTOR = "backLeftMotor";//2
    public static final String FRONT_RIGHT_MOTOR = "frontRightMotor";//1
    public static final String BACK_RIGHT_MOTOR = "backRightMotor";//0

    //intake Motor
    public static final String INTAKE_MOTOR = "intakeMotor";//

    // Motor Name - Slider
    public static final String SLIDER_MOTOR_LEFT = "sliderMotorLeft";
    public static final String SLIDER_MOTOR_RIGHT = "sliderMotorRight";

    // Motor speeds (Power settings)
    public static final double MAX_MOTOR_SPEED = 1.0;
    public static final double MIN_MOTOR_SPEED = 0.2;
    public static final double MOTOR_STOP = 0.0;


    // Slider Motor speeds (Power settings)
    public static final double SLIDER_SPEED = 0.8;
    public static final double SLIDER_HOLD_SPEED = 0.1;

    // Robot Movement Speeds (Drive system)
    public static final double DRIVE_SPEED = 0.5; // 50% speed
    public static final double TURN_SPEED = 0.3; // 30% turning speed

    //Slider Height
    public static final int SLIDE_MAX_HEIGHT = 3500;
    public static final int SLIDE_MIN_HEIGHT = 0;
    public static final int SLIDE_TIMEOUT = 4000;


    // Slider direction
    public static final int MOVE_DOWN = 0;
    public static final int MOVE_UP = 1;

    // Autonomous Mode Timeouts (in seconds)
    public static final double AUTONOMOUS_TIME_LIMIT = 30.0;

    //Ticks per inch from encoder
    static final double COUNTS_PER_INCH = 1440;

    // Color sequence
    public static final String VAL_21 = "";
    public static final String VAL_22 = "";
    public static final String VAL_23 = "";



}
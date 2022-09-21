//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
//
//import java.lang.Math;
//
//@TeleOp(name="Inception Smooth Drive", group="Iterative Opmode")
//public class SmoothInceptionDrive extends OpMode
//{
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotorEx leftFrontDrive = null;
//    private DcMotorEx rightFrontDrive = null;
//    private DcMotorEx leftBackDrive = null;
//    private DcMotorEx rightBackDrive = null;
//    private DcMotor intakeDrive = null;
//    private DcMotorEx outtakeDrive = null;
//    private Servo carousel1 = null;
//    private Servo carousel2 = null;
//    //    private Servo outtakeServo1 = null;
////    private Servo outtakeServo2 = null;
////    private DcMotorEx tunnelDrive =  null;
//    private DcMotorEx elevatorDrive1 = null;
//    private DcMotorEx elevatorDrive2 = null;
//    private int dpad_rightToggle = 0;
//    private boolean dpadr_pressed = false;
//    private int dpad_upToggle = 0;
//    private boolean dpadu_pressed = false;
//    private int dpad_leftToggle = 0;
//    private boolean dpadl_pressed = false;
//    private int dpad_downToggle = 0;
//    private boolean dpadd_pressed = false;
//    private int lastSpeed = 0;
//    private boolean sfModePast = false;
//    private int sfModeCounter = 0;
//    private Servo outtakeServo;
//    private Servo tapeRotateServo;
//    private Servo tapeUpDownServo;
//    private Servo tapeOutInServo;
//    private double counter = .000005;
//
//    @Override
//    public void init() {
//        telemetry.addData("Status", "Initialized");
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
//        leftBackDrive = hardwareMap.get(DcMotorEx.class, "right_front");
//        rightFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
//        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");
//        intakeDrive = hardwareMap.get(DcMotor.class, "intake");
////        outtakeDrive = hardwareMap.get(DcMotorEx.class, "outtake");
//        carousel1 = hardwareMap.get(Servo.class, "carousel1");
//        carousel2 = hardwareMap.get(Servo.class, "carousel2");
////        outtakeServo1 = hardwareMap.get(Servo.class, "outtake1");
////        outtakeServo2 = hardwareMap.get(Servo.class, "outtake2");
//        //tunnelDrive = hardwareMap.get(DcMotorEx.class, "tunnel");
//        elevatorDrive1 = hardwareMap.get(DcMotorEx.class, "elevator1");
//        elevatorDrive2 = hardwareMap.get(DcMotorEx.class, "elevator2");
//        outtakeServo = hardwareMap.get(Servo.class, "outtake");
//        tapeRotateServo = hardwareMap.get(Servo.class, "tapeRotateServo");
//        tapeUpDownServo = hardwareMap.get(Servo.class, "tapeUpDownServo");
//        tapeOutInServo = hardwareMap.get(Servo.class, "tapeOutInServo");
//
//
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        intakeDrive.setDirection(DcMotor.Direction.FORWARD);
////        outtakeDrive.setDirection(DcMotorEx.Direction.FORWARD);
////        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
////        outtakeServo1.setDirection(Servo.Direction.FORWARD);
////        outtakeServo2.setDirection(Servo.Direction.REVERSE);
//        //tunnelDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        elevatorDrive1.setDirection(DcMotorEx.Direction.FORWARD);
//        elevatorDrive2.setDirection(DcMotorEx.Direction.FORWARD);
//        outtakeServo.setDirection(Servo.Direction.FORWARD);
//
//
//        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
//
//        //Setting Zero Power behaviour
//        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        outtakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elevatorDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elevatorDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        //tunnelDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
//
//
//
//        //setting PID coefficients
//        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        rightFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
////        intakeDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        outtakeServo.setPosition(0);
////        outtakeDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
////        carouselMotor.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        //tunnelDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    @Override
//    public void init_loop() {
//    }
//
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//        runtime.reset();
//    }
//
//    @Override
//    public void loop() {
//
//    }
//
//    @Override
//    public void stop() {
//    }
//}

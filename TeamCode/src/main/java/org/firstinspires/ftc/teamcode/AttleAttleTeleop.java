//
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//
//
///**
// * This file contains an example of an iterative (Non-Linear) "OpMode".
// * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
// * The names of OpModes appear on the menu of the FTC Driver Station.
// * When an selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all iterative OpModes contain.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@TeleOp(name="Nikhil: AtlAtl_OP", group="Iterative Opmode")
//
//public class AttleAttleTeleop extends OpMode
//{
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotorEx leftFrontDrive = null;
//    private DcMotorEx rightFrontDrive = null;
//    private DcMotorEx leftBackDrive = null;
//    private DcMotorEx rightBackDrive = null;
//
//
//
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//        telemetry.addData("Status", "Initialized");
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
//        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
//        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
//        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");
//
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
//
//        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
//
//        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
//
//        //Setting Zero Power behaviour
//        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        //setting PID coefficients
//        //leftFrontDrive.setVelocityPIDFCoefficients(30, 0, 0, 0);
//        //rightFrontDrive.setVelocityPIDFCoefficients(30, 0, 0, 0);
//        //leftBackDrive.setVelocityPIDFCoefficients(30, 0, 0, 0);
//        //rightBackDrive.setVelocityPIDFCoefficients(30, 0, 0, 0);
//
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
//    /*
//     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//     */
//    @Override
//    public void loop() {
//        // Setup a variable for each drive wheel to save power level for telemetry
//        double leftFrontPower;
//        double rightFrontPower;
//        double leftBackPower;
//        double rightBackPower;
//
//        // Choose to drive using either Tank Mode, or POV Mode
//        // Comment out the method that's not used.  The default below is POV.
//
//        // POV Mode uses left stick to go forward, and right stick to turn.
//        // - This uses basic math to combine motions and is easier to drive straight.
//        double strafe = gamepad1.left_stick_x;
//        double drive = gamepad1.left_stick_y;
//        double turn  =  gamepad1.right_stick_x;
//        leftFrontPower   = drive - strafe - turn;
//        rightFrontPower  = drive - strafe + turn;
//        leftBackPower    = drive + strafe - turn;
//        rightBackPower   = drive + strafe + turn;
//
//        double maxValue = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
//
//        if (maxValue > 1) {
//            leftFrontPower /= maxValue;
//            rightFrontPower /= maxValue;
//            leftBackPower /= maxValue;
//            rightBackPower /= maxValue;
//        }
//        // Tank Mode uses one stick to control each wheel.
//        // - This requires no math, but it is hard to drive forward slowly and keep straight.
//        // leftPower  = -gamepad1.left_stick_y ;
//        // rightPower = -gamepad1.right_stick_y ;
//
//        // Send calculated velocity to wheels
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//
//        // Show the elapsed game time and wheel power.
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left front (%.2f), right front (%.2f), left back (%.2f), right back (%.2f)", leftFrontDrive.getPower(), rightFrontDrive.getPower(), leftBackDrive.getPower(), rightBackDrive.getPower());
//    }
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//    }
//
//}
//
//
//
//

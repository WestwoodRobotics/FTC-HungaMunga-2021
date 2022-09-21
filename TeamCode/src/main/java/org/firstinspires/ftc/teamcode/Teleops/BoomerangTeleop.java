package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="BoomerangTeleOp")
public class BoomerangTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx carousel = null;
    private DcMotor intake = null;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double carouselPower;
    boolean dpadPushed = false;
    boolean speedMode = false;
    private double intakePower;
    private boolean sfModePast = false;
    private int sfModeCounter = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_Front_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_Front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_Back_drive" );
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_Back_drive");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs when connected directly to the battery
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        carousel.setDirection(DcMotorEx.Direction.FORWARD);

        carousel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        double forward = (-gamepad1.left_stick_y);
        double strafe = gamepad1.left_stick_x;
        // Mulitiplied by .75 to get a slower turn
        double turn = (gamepad1.right_stick_x);
        boolean sfModeCurrent = gamepad1.b;

        leftFrontPower = (forward + strafe + turn);
        rightFrontPower = (forward - strafe - turn);
        leftBackPower = (forward - strafe + turn);
        rightBackPower = (forward + strafe - turn);
        double maxDrivetrainPower = Math.abs(Math.max(Math.max(Math.max(leftFrontPower, rightFrontPower), leftBackPower), rightBackPower));

        carouselPower = 0;

        if (gamepad1.dpad_right) {
            carouselPower = 500;
        } else if (gamepad1.dpad_left) {
            carouselPower = 500;
        } else {
            carouselPower = 0;
        }

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels

        if (!dpadPushed && gamepad1.dpad_up) {
            speedMode = !speedMode;
            dpadPushed = !dpadPushed;
        } else if (!gamepad1.dpad_up) {
            dpadPushed = !dpadPushed;
        }
        leftFrontPower /= maxDrivetrainPower;
        rightFrontPower /= maxDrivetrainPower;
        leftBackPower /= maxDrivetrainPower;
        rightBackPower /= maxDrivetrainPower;
        // Set motors
        if (speedMode) {
        } else {
            leftFrontPower *= 0.7;
            rightFrontPower *= 0.7;
            leftBackPower *= 0.7;
            rightBackPower *= 0.7;
        }

        if (sfModePast != sfModeCurrent) {
            sfModePast = sfModeCurrent;
            if (sfModeCounter == 2) {
                sfModeCounter = 0;
            } else {
                sfModeCounter += 1;
            }
        }

        if (sfModeCounter == 2) {
            leftFrontDrive.setVelocity(leftFrontPower * 1000);
            rightBackDrive.setVelocity(rightBackPower * 1000);
            leftBackDrive.setVelocity(leftBackPower * 1000);
            rightFrontDrive.setVelocity(rightFrontPower * 1000);
        } else {
            leftFrontDrive.setVelocity(leftFrontPower * 3000);
            rightBackDrive.setVelocity(rightBackPower * 3000);
            leftBackDrive.setVelocity(leftBackPower * 3000);
            rightFrontDrive.setVelocity(rightFrontPower * 3000);


            carousel.setVelocity(carouselPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f) leftBack (%.2f), rightBack (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.addData("Controller", "Forward (%.2f), Strafe (%.2f) Turn (%.2f)", forward, strafe, turn);
            telemetry.addData("Carousel", "Carousel: " + carouselPower);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

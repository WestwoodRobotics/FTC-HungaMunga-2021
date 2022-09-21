/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;



/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Nikhil: AtlAtl_OP", group="Iterative Opmode")

public class AttleAttleTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx carousel = null;
    private DcMotorEx intake = null;
    private DcMotorEx outtakeLift = null;
    private Servo box = null;

    public final static double box_home = 0.0;
    public final static double box_min_range = 0.0;
    public final static double box_max_range = 1.0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");
        carousel  = hardwareMap.get(DcMotorEx.class, "carousel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtakeLift = hardwareMap.get(DcMotorEx.class, "outtakelift");
        box = hardwareMap.get(Servo.class, "box");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        carousel.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeLift.setDirection(DcMotorEx.Direction.FORWARD);
        box.setPosition(box_home);

//        outtakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        outtakeLift.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        outtakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        outtakeLift.setTargetPosition(20000);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //Setting Zero Power behaviour
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        //setting PID coefficients
        //leftFrontDrive.setVelocityPIDFCoefficients(30, 0, 0, 0);
        //rightFrontDrive.setVelocityPIDFCoefficients(30, 0, 0, 0);
        //leftBackDrive.setVelocityPIDFCoefficients(30, 0, 0, 0);
        //rightBackDrive.setVelocityPIDFCoefficients(30, 0, 0, 0);

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
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double carouselPower;
        double intakePower;
        double outtakeLiftPower;



        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double strafe = gamepad1.left_stick_x;
        double drive = gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        boolean carouselMoveRight = gamepad1.right_bumper;
        boolean carouselMoveLeft = gamepad1.left_bumper;
        float intakeMoveIn = gamepad2.right_trigger;
        float intakeMoveOut = gamepad2.left_trigger;
        boolean outtakeLiftMoveUp = gamepad2.dpad_up;
        boolean outtakeLiftMoveDown = gamepad2.dpad_down;
        boolean boxMoveHome = gamepad2.dpad_left;
        boolean boxMoveOut = gamepad2.dpad_right;

        leftFrontPower   = -drive + strafe + turn;
        leftBackPower    = drive + strafe - turn;
        rightFrontPower  = -drive - strafe - turn;
        rightBackPower   = -drive + strafe - turn;

        if (intakeMoveIn > 0){
            intakePower = .75;
        }
        else if(intakeMoveOut > 0){
            intakePower = -.75;
        }
        else{
            intakePower = 0;
        }


        if(carouselMoveRight) {
            carouselPower = 0.8;
        }
        else if (carouselMoveLeft){
            carouselPower = -0.8;
        }
        else{
            carouselPower = 0;
        }
        if(carouselMoveLeft) {
            carouselPower = -0.8;
        }

        if(boxMoveHome) {
            box.setPosition(box_home);
        }
        if (boxMoveOut){
            box.setPosition(90);
        }

        if(outtakeLiftMoveUp) {
            //outtakeLift.setTargetPosition(200);
            outtakeLift.setPower(.5);
        }
        else if(outtakeLiftMoveDown) {
            //outtakeLift.setTargetPosition(-800);
            outtakeLift.setPower(-.5);
        }
        else{
            outtakeLift.setPower(0);
        }


        double maxValue = Math.max(Math.max(Math.abs(leftFrontPower),Math.abs(rightFrontPower)),Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (maxValue > 1) {
            leftFrontPower /= maxValue;
            rightFrontPower /= maxValue;
            leftBackPower /= maxValue;
            rightBackPower /= maxValue;
        }

        if(gamepad1.dpad_down) {
            leftFrontPower /= 0.6;
            rightFrontPower /= 0.6;
            leftBackPower /= 0.6;
            rightBackPower /= 0.6;

        }
        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated velocity to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        carousel.setPower(carouselPower);

        intake.setPower(intakePower);
        intake.setPower(intakePower);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left front (%.2f), right front (%.2f), left back (%.2f), right back (%.2f)", leftFrontDrive.getPower(), rightFrontDrive.getPower(), leftBackDrive.getPower(), rightBackDrive.getPower());
    }


    @Override
    public void stop() {
    }
//
}


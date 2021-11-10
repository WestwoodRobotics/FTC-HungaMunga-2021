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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Basic: AutomCaller", group="Linear Opmode")
public class AutomCaller extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;

//    private AutomMotorMethods LFDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
//    private AutomMotorMethods RFDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
//    private AutomMotorMethods LBDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
//    private AutomMotorMethods RBDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
    private AutomMotorMethods generalMotorMethods = new AutomMotorMethods(3500,
        0.0, 0.0, 0.0, 0.0);

    public void moveForwardDistance(double distanceInmm, AutomMotorMethods MotorMethods) {
        int conversionToTicks = MotorMethods.tickForDistance((int)distanceInmm);
        double moveForSecondAmount = MotorMethods.moveForSeconds(conversionToTicks);
        double startRunTime = runtime.seconds();
        while (runtime.seconds() <= startRunTime + moveForSecondAmount ) {
            leftFrontDrive.setVelocity(1 * 3500);
            rightBackDrive.setVelocity(1 * 3500);
            leftBackDrive.setVelocity(1 * 3500);
            rightFrontDrive.setVelocity(1 * 3500);
        }
        // MotorMethods.updatePos();
    }

    public void rotateToAngle(double angle, AutomMotorMethods MotorMethods) {
        int getDistanceToMove = MotorMethods.angleToDistance(angle);
        int conversionToTicks = MotorMethods.tickForDistance(getDistanceToMove);
        double moveForSecondAmount = MotorMethods.moveForSeconds(conversionToTicks);
        MotorMethods.updateAngle(angle);
        double startRunTime = runtime.seconds();
        int LFDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
                "leftFront");
        int LBDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
                "leftBack");
        int RFDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
                "rightFront");
        int RBDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
                "rightBack");
        while (runtime.seconds() <= startRunTime + moveForSecondAmount) {
            leftFrontDrive.setVelocity(LFDVelocity*3500);
            leftBackDrive.setVelocity(LBDVelocity*3500);
            rightFrontDrive.setVelocity(RFDVelocity*3500);
            rightBackDrive.setVelocity(RBDVelocity*3500);
        }
    }

    public void stopMotorSpin() {
        leftFrontDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double matSize = 24*25.4;

        // run until the end of the match (driver presses STOP)
        while (runtime.seconds() < 30) {


            moveForwardDistance(matSize, generalMotorMethods);
            telemetry.addData("Motors", "left front (%.2f)", leftFrontDrive.getVelocity());
            telemetry.addData("Motors", "right front (%.2f)", rightFrontDrive.getVelocity());
            telemetry.addData("Motors", "left back (%.2f)", leftBackDrive.getVelocity());
            telemetry.addData("Motors", "right back (%.2f)", rightBackDrive.getVelocity());
            stopMotorSpin();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

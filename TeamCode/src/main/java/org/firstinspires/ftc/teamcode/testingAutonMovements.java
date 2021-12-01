package org.firstinspires.ftc.teamcode;
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;


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

@Autonomous(name="Basic: testingAutonMovements", group="Linear Opmode")
public class testingAutonMovements extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private double globalMovementTimer = 0;
    final int robotVelocity = 2800;
    private Servo carouselServo = null;

    //    private AutomMotorMethods LFDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
//    private AutomMotorMethods RFDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
//    private AutomMotorMethods LBDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
//    private AutomMotorMethods RBDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
    private AutomMotorMethods generalMotorMethods = new AutomMotorMethods(560,
            0.0, 0.0, 0.0, 0.0, robotVelocity);

    public void moveMotorsFor(double timeDuration, double velocity, String forwardOrBackwards) {
        if (forwardOrBackwards == "FORWARDS") {
            leftFrontDrive.setVelocity(velocity);
            rightBackDrive.setVelocity(velocity);
            leftBackDrive.setVelocity(velocity);
            rightFrontDrive.setVelocity(velocity);
        }
        else if (forwardOrBackwards == "BACKWARDS") {
            leftFrontDrive.setVelocity(-velocity);
            rightBackDrive.setVelocity(-velocity);
            leftBackDrive.setVelocity(-velocity);
            rightFrontDrive.setVelocity(-velocity);
        }

        globalMovementTimer += timeDuration;

        while (runtime.seconds() < globalMovementTimer) {
            telemetry.addData("Motors", "left front (%.2f)", leftFrontDrive.getVelocity());
            telemetry.addData("Motors", "right front (%.2f)", rightFrontDrive.getVelocity());
            telemetry.addData("Motors", "left back (%.2f)", leftBackDrive.getVelocity());
            telemetry.addData("Motors", "right back (%.2f)", rightBackDrive.getVelocity());
            telemetry.addData("globaTimer", "globalTime: " + globalMovementTimer);
            telemetry.addData("Time Duration", "currentStageDuration: " + velocity);
            telemetry.addData("Time", "time: " + runtime.seconds());
            telemetry.update();
        }
    }

    public void rotateMotorToAngles(double timeDuration, double velocity, int leftFrontDirection, int leftBackDirection, int rightFrontDirection,
                                    int rightBackDirection) {
        leftFrontDrive.setVelocity(leftFrontDirection*velocity);
        rightBackDrive.setVelocity(rightBackDirection*velocity);
        leftBackDrive.setVelocity(leftBackDirection*velocity);
        rightFrontDrive.setVelocity(rightFrontDirection*velocity);


        globalMovementTimer += timeDuration;

        while (runtime.seconds() < globalMovementTimer) {
            telemetry.addData("Motors", "left front (%.2f)", leftFrontDrive.getVelocity());
            telemetry.addData("Motors", "right front (%.2f)", rightFrontDrive.getVelocity());
            telemetry.addData("Motors", "left back (%.2f)", leftBackDrive.getVelocity());
            telemetry.addData("Motors", "right back (%.2f)", rightBackDrive.getVelocity());
            telemetry.addData("globaTimer", "globalTime: " + globalMovementTimer);
            telemetry.addData("Time Duration", "currentStageDuration: " + velocity);
            telemetry.addData("Time", "time: " + runtime.seconds());
            telemetry.update();
        }
    }

    public void moveForwardOrBackwardsDistance(double distanceInmm, AutomMotorMethods MotorMethods, String Direction) {
        int conversionToTicks = MotorMethods.tickForDistance((int)distanceInmm);
        HashMap<String, Double> stageHolder = MotorMethods.generateMotionProfile(conversionToTicks,
                robotVelocity);
        double stageAmount = stageHolder.get("numStages");
        if (stageAmount == 1) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
        }
        else if (stageAmount == 2) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
        }
        else if (stageAmount == 3) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
        }
        else if (stageAmount == 4) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
        }
        else if (stageAmount == 5) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
        }
        else if (stageAmount == 6) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
        }
        else if (stageAmount == 7) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
        }
        else if (stageAmount == 8) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
        }
        else if (stageAmount == 9) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
        }
        else if (stageAmount == 10) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
        }
        else if (stageAmount == 11) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
        }
        else if (stageAmount == 12) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), Direction);
        }
        else if (stageAmount == 13) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), Direction);
        }
        else if (stageAmount == 14) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), Direction);
        }
        else if (stageAmount == 15) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), Direction);
        }
        else if (stageAmount == 16) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), Direction);
        }
        else if (stageAmount == 17) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), Direction);
        }
        else if (stageAmount == 18) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage18Duration"), stageHolder.get("stage18Velocity"), Direction);
        }
        else if (stageAmount == 19) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage18Duration"), stageHolder.get("stage18Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage19Duration"), stageHolder.get("stage19Velocity"), Direction);
        }
        else if (stageAmount == 20) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage18Duration"), stageHolder.get("stage18Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage19Duration"), stageHolder.get("stage19Velocity"), Direction);
            moveMotorsFor(stageHolder.get("stage20Duration"), stageHolder.get("stage20Velocity"), Direction);

        }


        leftFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);


        // MotorMethods.updatePos();
    }

    public void rotateToAngle(double angle, AutomMotorMethods MotorMethods) {
        int getDistanceToMove = MotorMethods.angleToDistance(angle);
        int conversionToTicks = MotorMethods.tickForDistance(getDistanceToMove);
        MotorMethods.updateAngle(angle);
        int LFDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
                "leftFront");
        int LBDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
                "leftBack");
        int RFDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
                "rightFront");
        int RBDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
                "rightBack");

        HashMap<String, Double> stageHolder = MotorMethods.generateMotionProfile(conversionToTicks,
                robotVelocity);

        double stageAmount = stageHolder.get("numStages");
        if (stageAmount == 1) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 2) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 3) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 4) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 5) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 6) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 7) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 8) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 9) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 10) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 11) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 12) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 13) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 14) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 15) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 16) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 17) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 18) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage18Duration"), stageHolder.get("stage18Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 19) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage18Duration"), stageHolder.get("stage18Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage19Duration"), stageHolder.get("stage19Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
        }
        else if (stageAmount == 20) {
            rotateMotorToAngles(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage18Duration"), stageHolder.get("stage18Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage19Duration"), stageHolder.get("stage19Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);
            rotateMotorToAngles(stageHolder.get("stage20Duration"), stageHolder.get("stage20Velocity"), LFDVelocity, LBDVelocity, RFDVelocity, RBDVelocity);

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
        carouselServo = hardwareMap.get(Servo.class, "carousel");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        carouselServo.setDirection(Servo.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setVelocityPIDFCoefficients(20, 0, 0, 0);
        rightFrontDrive.setVelocityPIDFCoefficients(20, 0, 0, 0);
        leftBackDrive.setVelocityPIDFCoefficients(20, 0, 0, 0);
        rightBackDrive.setVelocityPIDFCoefficients(20, 0, 0, 0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


//        double matSize = 2*(24*25.4);
        double matSize = 7.5398223686155035+15.079644737231007+22.61946710584651+30.159289474462014+37.69911184307752+45.23893421169302+52.778756580308524+60.31857894892403+67.85840131753953+75.39822368615503+82.93804605477054+90.47786842338604+20.465232143384938;
        double matSize2 = 6*((24+7)*25.4);


        // run until the end of the match (driver presses STOP)

        moveForwardOrBackwardsDistance(matSize2, generalMotorMethods, "FORWARDS");







        // stopMotorSpin();

    }
}




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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

//@Autonomous(name="Basic: testAutomSpeeds", group="Linear Opmode")
public class testingMotorSpeeds extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx testMotor = null;
    private double globalMovementTimer = 0;
    final int robotVelocity = 2800;

    //    private AutomMotorMethods LFDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
//    private AutomMotorMethods RFDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
//    private AutomMotorMethods LBDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
//    private AutomMotorMethods RBDMethods = new AutomMotorMethods(3500, 0.0,
//            0.0);
    private AutomMotorMethods generalMotorMethods = new AutomMotorMethods(3500,
            0.0, 0.0, 0.0, 0.0, robotVelocity);

    public void moveMotorsFor(double timeDuration, double velocity, String forwardOrBackwards) {
        if (forwardOrBackwards == "FORWARDS") {
            testMotor.setVelocity(velocity);
        }
        else if (forwardOrBackwards == "BACKWARDS") {
            testMotor.setVelocity(-velocity);
        }
        globalMovementTimer += timeDuration;

        while (runtime.seconds() < globalMovementTimer) {
            telemetry.addData("Motors", "left front (%.2f)", testMotor.getVelocity());
            telemetry.addData("globaTimer", "globalTime: " + globalMovementTimer);
            telemetry.addData("Time Duration", "currentStageDuration: " + velocity);
            telemetry.addData("Time", "time: " + runtime.seconds());
            telemetry.update();
        }
    }

    public void moveForwardDistance(double distanceInmm, AutomMotorMethods MotorMethods) {
        int conversionToTicks = MotorMethods.tickForDistance((int)distanceInmm);
        HashMap<String, Double> stageHolder = MotorMethods.generateMotionProfile(conversionToTicks,
                robotVelocity);
        double stageAmount = stageHolder.get("numStages");
        if (stageAmount == 1) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
        }
        else if (stageAmount == 2) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
        }
        else if (stageAmount == 3) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
        }
        else if (stageAmount == 4) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
        }
        else if (stageAmount == 5) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
        }
        else if (stageAmount == 6) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
        }
        else if (stageAmount == 7) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
        }
        else if (stageAmount == 8) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
        }
        else if (stageAmount == 9) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
        }
        else if (stageAmount == 10) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
        }
        else if (stageAmount == 11) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
        }
        else if (stageAmount == 12) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), "FORWARDS");
        }
        else if (stageAmount == 13) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), "FORWARDS");
        }
        else if (stageAmount == 14) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), "FORWARDS");
        }
        else if (stageAmount == 15) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), "FORWARDS");
        }
        else if (stageAmount == 16) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), "FORWARDS");
        }
        else if (stageAmount == 17) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), "FORWARDS");
        }
        else if (stageAmount == 18) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage18Duration"), stageHolder.get("stage18Velocity"), "FORWARDS");
        }
        else if (stageAmount == 19) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage18Duration"), stageHolder.get("stage18Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage19Duration"), stageHolder.get("stage19Velocity"), "FORWARDS");
        }
        else if (stageAmount == 20) {
            moveMotorsFor(stageHolder.get("stage1Duration"), stageHolder.get("stage1Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage2Duration"), stageHolder.get("stage2Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage3Duration"), stageHolder.get("stage3Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage4Duration"), stageHolder.get("stage4Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage5Duration"), stageHolder.get("stage5Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage6Duration"), stageHolder.get("stage6Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage7Duration"), stageHolder.get("stage7Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage8Duration"), stageHolder.get("stage8Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage9Duration"), stageHolder.get("stage9Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage10Duration"), stageHolder.get("stage10Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage11Duration"), stageHolder.get("stage11Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage12Duration"), stageHolder.get("stage12Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage13Duration"), stageHolder.get("stage13Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage14Duration"), stageHolder.get("stage14Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage15Duration"), stageHolder.get("stage15Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage16Duration"), stageHolder.get("stage16Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage17Duration"), stageHolder.get("stage17Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage18Duration"), stageHolder.get("stage18Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage19Duration"), stageHolder.get("stage19Velocity"), "FORWARDS");
            moveMotorsFor(stageHolder.get("stage20Duration"), stageHolder.get("stage20Velocity"), "FORWARDS");

        }

        testMotor.setVelocity(0);

        // MotorMethods.updatePos();
    }

    public void rotateToAngle(double angle, AutomMotorMethods MotorMethods) {
//        int getDistanceToMove = MotorMethods.angleToDistance(angle);
//        int conversionToTicks = MotorMethods.tickForDistance(getDistanceToMove);
//        double moveForSecondAmount = MotorMethods.moveForSeconds(conversionToTicks);
//        MotorMethods.updateAngle(angle);
//        globalMovementTimer += moveForSecondAmount;
//        int LFDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
//                "leftFront");
//        int LBDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
//                "leftBack");
//        int RFDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
//                "rightFront");
//        int RBDVelocity = MotorMethods.findOptimumMovementForRobotRotation(angle,
//                "rightBack");
//        while (runtime.seconds() <=  globalMovementTimer) {
//            testMotor.setVelocity(LFDVelocity*3500);
//        }
    }

    public void stopMotorSpin() {
        testMotor.setVelocity(0);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        testMotor  = hardwareMap.get(DcMotorEx.class, "intake");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        testMotor.setDirection(DcMotorEx.Direction.FORWARD);

        testMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        testMotor.setVelocityPIDFCoefficients(15, 0, 0, 0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double matSize = 24*25.4;

        // run until the end of the match (driver presses STOP)
        moveForwardDistance(matSize, generalMotorMethods);

        // stopMotorSpin();

    }
}

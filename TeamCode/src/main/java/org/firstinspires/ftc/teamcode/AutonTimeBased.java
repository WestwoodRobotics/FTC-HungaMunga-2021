package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Anaya: AutonTimeBased", group="Linear Opmode")
public class AutonTimeBased extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;

    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);

        AutonMethods methods = new AutonMethods(this, telemetry, leftFrontDrive,  rightFrontDrive, leftBackDrive, rightBackDrive);

        while (runtime.seconds() < 30) {
            methods.goForward(2, 1100);

        }


//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left front (%.2f)", leftFrontDrive.getVelocity());
//        telemetry.addData("Motors", "right front (%.2f)", rightFrontDrive.getVelocity());
//        telemetry.addData("Motors", "left back (%.2f)", leftBackDrive.getVelocity());
//        telemetry.addData("Motors", "right back (%.2f)", rightBackDrive.getVelocity());
//
//        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.placements;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonMethods;

@Autonomous(name="Anaya: AutonTimeBased1", group="Linear Opmode")
public class placement1 extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx intakeDrive = null;
    private Servo carouselServo = null;
    private Servo outtakeServo1 = null;
    private Servo outtakeServo2 = null;

    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");
        intakeDrive = hardwareMap.get(DcMotorEx.class, "intake");
        carouselServo = hardwareMap.get(Servo.class, "carousel");
        outtakeServo1 = hardwareMap.get(Servo.class, "outtake1");
        outtakeServo2 = hardwareMap.get(Servo.class, "outtake2");

        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        intakeDrive.setDirection(DcMotorEx.Direction.REVERSE);
        carouselServo.setDirection(Servo.Direction.FORWARD);
        outtakeServo1.setDirection(Servo.Direction.FORWARD);
        outtakeServo2.setDirection(Servo.Direction.REVERSE);



        AutonMethods methods = new AutonMethods(this, telemetry, leftFrontDrive,  rightFrontDrive, leftBackDrive, rightBackDrive, intakeDrive, carouselServo);

        while (runtime.seconds() < 30) {
            methods.rightTurn(.5, 3500 );
            methods.goForward(.2, 3500);
            methods.turnCarousel(5);
            methods.rightTurn(1, 3500);
            methods.leftStrafe(1, 3500);
            methods.goForward(5, 1100);
        }




        //displays the telemetry, don't know if we are using thism
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f)", leftFrontDrive.getVelocity());
        telemetry.addData("Motors", "right front (%.2f)", rightFrontDrive.getVelocity());
        telemetry.addData("Motors", "left back (%.2f)", leftBackDrive.getVelocity());
        telemetry.addData("Motors", "right back (%.2f)", rightBackDrive.getVelocity());
        telemetry.addData("Boolean", "carousel(%b)", carouselServo);
        telemetry.addData("boolean", "outtake(%b", outtakeServo1);
        telemetry.addData("Boolean", "outtake(%b)", outtakeServo2);

        telemetry.update();
    }
}
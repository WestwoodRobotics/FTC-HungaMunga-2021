package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;

@Autonomous(name="TestingAutonElevatorControl", group="Linear Opmode")
public class TestingAutonElevatorControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double globalMovementTimer = 0;
    private DistanceSensor sensorRange;
    private Servo outtakeServo1 = null;
    private Servo outtakeServo2 = null;
    private DcMotorEx elevatorDrive = null;
    private DistanceSensor elevatorSensor;
    double thirdLevel = 39.5;
    double secondLevel = 26.5;
    double firstLevel = 19.5;
    // distance to 3 level is 37.7 cm
    // distance to 2 level is 26.5 cm
    // distance to 1 level is 19.5 cm
    private int servoMovementDuration = 2000; // set in milliseconds

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        elevatorSensor = hardwareMap.get(DistanceSensor.class, "elevatorSensor");
        outtakeServo1 = hardwareMap.get(Servo.class, "outtake1");
        outtakeServo2 = hardwareMap.get(Servo.class, "outtake2");
        elevatorDrive = hardwareMap.get(DcMotorEx.class, "elevator");


        outtakeServo1.setDirection(Servo.Direction.FORWARD);
        outtakeServo2.setDirection(Servo.Direction.REVERSE);
        elevatorDrive.setDirection(DcMotorEx.Direction.REVERSE);

        elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)

        // moving to the third level to score
        elevatorDrive.setVelocity(1200);

        while (elevatorSensor.getDistance(DistanceUnit.CM) < firstLevel) {
            telemetry.addData("range", String.format("%.01f cm", elevatorSensor.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }

        elevatorDrive.setVelocity(0);

        while (runtime.seconds() < 3) {

        }
        outtakeServo1.setPosition(1);
        outtakeServo2.setPosition(1);
        sleep(servoMovementDuration);
    }
}
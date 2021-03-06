package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonMethods {
    private LinearOpMode opmode;
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx intakeDrive = null;
    private Servo carouselServo = null;
    private DcMotorEx tunnelDrive =  null;
    private DcMotorEx elevatorDrive = null;

    public AutonMethods (LinearOpMode opmode,
                         Telemetry telemetry,
                         DcMotorEx leftFrontDrive,
                         DcMotorEx rightFrontDrive,
                         DcMotorEx leftBackDrive,
                         DcMotorEx rightBackDrive,
                         DcMotorEx intakeDrive,
                         Servo carouselServo,
                         DcMotorEx tunnelDrive,
                         DcMotorEx elevatorDrive) {

        this.opmode = opmode;
        this.telemetry = telemetry;
        this.leftFrontDrive = leftFrontDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightBackDrive = rightBackDrive;
        this.intakeDrive = intakeDrive;
        this.carouselServo = carouselServo;
        this.tunnelDrive = tunnelDrive;
        this.elevatorDrive = elevatorDrive;
    }

    //go forward for time
    public void goForward(double seconds, double velocity) {
        runtime.reset();
        leftFrontDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);

        while (runtime.seconds() < seconds ){

        }

        leftFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);
    }

    //go backwards for time
    public void goBackwards(double seconds, double velocity) {
        runtime.reset();
        leftFrontDrive.setVelocity(-velocity);
        rightBackDrive.setVelocity(-velocity);
        leftBackDrive.setVelocity(-velocity);
        rightFrontDrive.setVelocity(-velocity);

        while (runtime.seconds() < seconds ){

        }

        leftFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);
    }

    //strafe right for time
    public void rightStrafe(double seconds, double velocity) {
        runtime.reset();
        leftFrontDrive.setVelocity(-velocity);
        rightBackDrive.setVelocity(-velocity);
        leftBackDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);

        while (runtime.seconds() < seconds ){

        }

        leftFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);
    }

    //strafe left for time
    public void leftStrafe(double seconds, double velocity) {
        runtime.reset();
        leftFrontDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(-velocity);
        rightFrontDrive.setVelocity(-velocity);

        while (runtime.seconds() < seconds ){

        }

        leftFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);
    }

    //turn right for time
    public void rightTurn(double seconds, double velocity) {
        runtime.reset();
        leftFrontDrive.setVelocity(-velocity);
        rightBackDrive.setVelocity(-velocity);
        leftBackDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);

        while (runtime.seconds() < seconds ){

        }

        leftFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);
    }

    //turn left for time
    public void leftTurn(double seconds, double velocity) {
        runtime.reset();
        leftFrontDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(-velocity);
        rightFrontDrive.setVelocity(-velocity);

        while (runtime.seconds() < seconds ){

        }

        leftFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);
    }
    //go forward for time
    public void intakeIn(double seconds, double velocity) {
        runtime.reset();
        intakeDrive.setVelocity(-velocity);
        tunnelDrive.setVelocity(-velocity);
        while (runtime.seconds() < seconds ){

        }


        intakeDrive.setVelocity(0);
    }
    //go forward for time
    public void intakeOut(double seconds, double velocity) {
        runtime.reset();
        intakeDrive.setVelocity(velocity);

        while (runtime.seconds() < seconds ){

        }

        intakeDrive.setVelocity(0);
    }
    //go forward for time
    public void turnCarousel (double seconds) {
        runtime.reset();
        carouselServo.setPosition(1);

        while (runtime.seconds() < seconds ){

        }

        carouselServo.setPosition(.5);


    }
    public void elevatorUp (double seconds) {
        runtime.reset();
        elevatorDrive.setVelocity(1);

        while (runtime.seconds() < seconds ){

        }

        elevatorDrive.setVelocity(0);

    }
    public void elevatorDown (double seconds) {
        runtime.reset();
        elevatorDrive.setVelocity(-1);
        while (runtime.seconds() < seconds ){

        }

        elevatorDrive.setVelocity(0);

    }



    public int motionProfileGenerator(int desiredTicksPerSecond) {
        return 0;
    }
}
package org.firstinspires.ftc.teamcode.placements;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonMethods;

@Autonomous(name="HungaMunga: AutonTimeBased3", group="Linear Opmode")
public class placement3 extends LinearOpMode {

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
    public void runOpMode() throws InterruptedException {

    }

}
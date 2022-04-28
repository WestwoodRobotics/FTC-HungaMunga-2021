package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.HashMap;

@Autonomous(name="AtlAtlMoveForwardAuton", group="Linear Opmode")
public class AtlAtlMoveForwardAuton extends LinearOpMode {
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs when connected directly to the battery
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);

        leftFrontDrive.setVelocity(3000);
        leftBackDrive.setVelocity(3000);
        rightBackDrive.setVelocity(3000);
        rightFrontDrive.setVelocity(3000);

        while (runtime.seconds() < 2) {

        }
    }
}

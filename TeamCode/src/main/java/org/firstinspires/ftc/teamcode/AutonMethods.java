package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonMethods {
    private LinearOpMode opmode;
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    public AutonMethods  (LinearOpMode opmode,
                          Telemetry telemetry,
                          DcMotorEx leftFrontDrive,
                          DcMotorEx rightFrontDrive,
                          DcMotorEx leftBackDrive,
                          DcMotorEx rightBackDrive) {
        this.opmode = opmode;
        this.telemetry = telemetry;
        this.leftFrontDrive = leftFrontDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightBackDrive = rightBackDrive;
        }
        //go forward for time
        public void goForward (double seconds, double velocity) {
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


}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
@TeleOp(name = "TestingElevatorDistances", group = "Iterative Opmode")
public class EstablishElavatorDistances extends LinearOpMode {

    private DistanceSensor elevatorSensor;
    private Servo outtakeServo1 = null;
    private Servo outtakeServo2 = null;
    private DcMotorEx elevatorDrive = null;
    // distance to 3 level is 37.7 cm
    // distance to 2 level is 26.5 cm
    // distance to 1 level is 19.5 cm

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        elevatorSensor = hardwareMap.get(DistanceSensor.class, "elevatorSensor");
        outtakeServo1 = hardwareMap.get(Servo.class, "outtake1");
        outtakeServo2 = hardwareMap.get(Servo.class, "outtake2");
        elevatorDrive = hardwareMap.get(DcMotorEx.class, "elevator");

        outtakeServo1.setDirection(Servo.Direction.FORWARD);
        outtakeServo2.setDirection(Servo.Direction.REVERSE);
        elevatorDrive.setDirection(DcMotorEx.Direction.REVERSE);

        elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)elevatorSensor;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            boolean outtakeIn = gamepad1.left_bumper;
            boolean outtakeOut = gamepad1.right_bumper;
            boolean elevatorUp = gamepad1.dpad_up;
            boolean elevatorDown = gamepad1.dpad_down;

            if (outtakeOut== true) {
                outtakeServo1.setPosition(1);
                outtakeServo2.setPosition(1);
            }
            else if (outtakeIn == true) {
                outtakeServo1.setPosition(0);
                outtakeServo2.setPosition(0);
            }
            else {
                outtakeServo1.setPosition(.5);
                outtakeServo2.setPosition(.5);
            }

            if (elevatorUp == true) {
                elevatorDrive.setVelocity(1200);
            }
            else if (elevatorDown == true) {
                elevatorDrive.setVelocity(-1200);
            }
            else{
                elevatorDrive.setVelocity(0);
            }

            // generic DistanceSensor methods.
            telemetry.addData("deviceName",elevatorSensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", elevatorSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", elevatorSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", elevatorSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", elevatorSensor.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();

        }
    }

}

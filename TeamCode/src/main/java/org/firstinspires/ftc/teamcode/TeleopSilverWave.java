package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Team Silver Wave", group="Iterative Opmode")
public class TeleopSilverWave extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Servo spinner = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        spinner = hardwareMap.get(Servo.class, "spinner");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        spinner.setPosition(1);
    }
}

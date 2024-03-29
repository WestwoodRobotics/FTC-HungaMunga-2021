package org.firstinspires.ftc.teamcode.VisionCode;

import android.hardware.camera2.CameraCharacteristics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.bots.DummyBot;
import org.firstinspires.ftc.teamcode.VisionCode.Detector;
import org.firstinspires.ftc.teamcode.VisionCode.tfrec.classification.Classifier;

import java.util.List;

//Opmode for quick testing of motors
@TeleOp(name="TFDetector", group="Robot15173")
public class DetectorTest extends LinearOpMode{

    // Declare OpMode members.
    private Detector tfDetector = null;
    private ElapsedTime runtime = new ElapsedTime();

    private static String MODEL_FILE_NAME = "PurpleGreen.tflite";
    private static String LABEL_FILE_NAME = "PurpleGreenLabels.txt";
    private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;

    @Override
    public void runOpMode() {
        try {
            try {
                tfDetector = new Detector(MODEl_TYPE, MODEL_FILE_NAME, LABEL_FILE_NAME, hardwareMap.appContext, telemetry);
                tfDetector.activate();
            }
            catch (Exception ex){
                telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
                sleep(3000);
                return;
            }

            telemetry.addData("Detector", "Ready");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0){
                    telemetry.addData("Info", "No results");
                }
                else {
                    for (Classifier.Recognition r : results) {
                        String item = String.format("%s: %.2f", r.getTitle(), r.getConfidence());
                        telemetry.addData("Found", item);
                    }
                }
                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        }
        finally {
            if (tfDetector != null){
                tfDetector.stopProcessing();
            }
        }
    }
}

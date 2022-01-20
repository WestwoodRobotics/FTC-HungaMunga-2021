package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
//@Autonomous(name = "IdentifyingCustomObject", group = "Linear Opmode")
public class identifyingCustomObject extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Servo testServo = null;
    private double globalMovementTimer = 0;
    private double servoMovementDuration = 2;

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Afl+RML/////AAABmaMhjqdoa0n5o+LOBUlDi2FnRR3PvjXoX9GXBYOqhEzqDK5EKlhfk7DF9xzcZYPAYSpcyiB" +
                    "sR9g89nQniC//pOf07VI7X8ajr5AWIGShrW3kPrD1uIlLG7IXWGA6AdL2onK51Nebvu7Qqim+BPqJRa" +
                    "gq70nEvg5DrPujktWjnuu8TA3Mm7V9Ir/XYNqhyrAxxfm/yVwUUvP6l3Km+T9qvu8ezX628BmdMXhM4" +
                    "cEoEHn1paQ6if7ZyJg4c6EPpAh96SKfBLtQyb1Yk+dM+0jtXKeLPfOqfdVJ5fyckkN04f2oQtJcZE5R" +
                    "VaVDeszXltcxNLyK1EpYCgXi5fmS+bpqPUziQtHyWQSf4uwqKlI5Eo/x";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    private WebcamName webcam1;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

    public boolean findDuck(int triesLeft) {
        updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            return true;
        }
        else if (triesLeft <= 0) {
            return false;
        }
        else {
//            sleep(250);
            return findDuck(triesLeft-1);
        }
    }


//    public boolean foundDuck() {
//        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//        boolean foundDuck = false;
//        if ( updatedRecognitions != null) {
//            int recognitionSize = updatedRecognitions.size();
//            for (Recognition recognition : updatedRecognitions) {
//                if (recognition.getLabel().equals("Duck")) {
//                    foundDuck = true;
//                }
//            }
//        }
//        else {
//            updatedRecognitions = tfod.getUpdatedRecognitions();
//            if ( updatedRecognitions != null) {
//                int recognitionSize = updatedRecognitions.size();
//                for (Recognition recognition : updatedRecognitions) {
//                    if (recognition.getLabel().equals("Duck")) {
//                        foundDuck = true;
//                    }
//                }
//            }
//        }
//        return foundDuck;
//    }



    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.2, 16.0/9.0);
        }

        testServo = hardwareMap.get(Servo.class, "outtake2");
        testServo.setDirection(Servo.Direction.FORWARD);
        testServo.setPosition(0.0);
        sleep(5000);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        testServo.setPosition(.5);
        sleep(5000);

        testServo.setPosition(1.0);
        sleep(5000);


//        testServo.setPosition(0); // needs to be tested for correct position
//        while (runtime.seconds() < globalMovementTimer + servoMovementDuration) { }
//        boolean foundIt = this.foundDuck();
//        if (foundIt == false) {
//            testServo.setPosition(.5); // needs to be tested for correct position
//            while (runtime.seconds() < globalMovementTimer + servoMovementDuration) { }
//            foundIt = this.foundDuck();
//            if (foundIt == false) {
//                testServo.setPosition(0); // needs to be tested for correct position
//                while (runtime.seconds() < globalMovementTimer + servoMovementDuration) { }
//                foundIt = this.foundDuck();
//            }
//        }



//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//
//                        // step through the list of recognitions and display boundary info.
//                        int i = 0;
//                        for (Recognition recognition : updatedRecognitions) {
//                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                    recognition.getLeft(), recognition.getTop());
//                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                    recognition.getRight(), recognition.getBottom());
//                            i++;
//                        }
//                        telemetry.update();
//                    }
//                }
//            }
//        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}


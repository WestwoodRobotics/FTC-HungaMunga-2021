package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
@TeleOp(name = "DetectingMe", group = "Concept")
public class DetectingMe extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double globalMovementTimer = 0;
    private double servoMovementDuration = 2;
    // a wheel will be moved accordingly depending on where the camera detected the ducky
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;

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
    private static final String TFOD_MODEL_ASSET = "MeAndCalc.tflite";
    private static final String[] LABELS = {
            "Me",
            "Calculator"
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
    // put your own key my key might be all used up by now
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

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public List<Recognition> findDuck(int tries) {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        int foundDucky = foundDuck(updatedRecognitions);
        if (foundDucky != 0) {
            return updatedRecognitions;
        }
        else if (tries <= 0) {
            return null;
        }
        else {
//            sleep(250);
            return findDuck(tries-1);
        }
    }

    public int foundDuck(List<Recognition> updatedRecognitions) {
        int index = 0;
        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                String currentLabel = recognition.getLabel();
                if (currentLabel.equals("Duck")) {
                    return index+1;
                }
                index += 1;
            }
        }
        return 0;
    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);

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
            tfod.setZoom(1, 16.0/9.0);
            // 1 zoom mean you use the whole camera(play around with it to find a good zoom)
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        List<Recognition> updatedRecognitions;



        // screen being split in three
        float line1 = 426; // put pixel value++
        float line2 = 853; // put pixel value
        // image our screen has 30 pixel left to right, this is how it would be split
        // camera is 1280*720 pixels

        updatedRecognitions = findDuck(10);
        int duckIndex = foundDuck(updatedRecognitions);
        telemetry.addData("find duck?", duckIndex);
        telemetry.update();
        sleep(3000);


        int i = 0;
        for (Recognition recognition : updatedRecognitions) {
            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    recognition.getRight(), recognition.getBottom());
            i++;
        }
        telemetry.update();
        sleep(3000);

        if (updatedRecognitions != null && duckIndex != 0) {
            /* a variable storing the duck right position
            with this we can just check fro left to right on the camera to know
            on which section of the camera the ducky is in
             */
            float duckRight = updatedRecognitions.get(duckIndex-1).getRight();


            /* example of what code does next
            || = line1
            | = line2
             nothing1  ||  nothing2  | duck
             the code has the right pos of the duck
             meaning we first check if the right position is less than || position
             then we check if the right position is less than | position
             and lastly if it wasn't less than any of does two and it's position is
             in the last part
             */
            if (duckRight < line1) {
                leftBackDrive.setVelocity(3500);
                telemetry.addData("duckRight", duckRight);
                telemetry.update();
                // sleep does in milliseconds - 1 second = 1000 seconds
                sleep(3000);
            }
            else if (duckRight < line2) {
                leftFrontDrive.setVelocity(3500);
                telemetry.addData("duckRight", duckRight);
                telemetry.update();
                sleep(3000);
            }
            else {
                rightBackDrive.setVelocity(3500);
                telemetry.addData("duckRight", duckRight);
                telemetry.update();
                sleep(3000);
            }
        }

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
        tfodParameters.minResultConfidence = 0.3f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}

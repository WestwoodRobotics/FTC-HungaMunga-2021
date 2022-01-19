package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class AutonomousDuckFinderUsingServo extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private double globalMovementTimer = 0;
    private int servoMovementDuration = 2000; // set in milliseconds
    double servoMovement = .3;// from 0 - 1
    private Servo cameraServo = null;
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
    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

    public boolean findDuck(int tries) {
        updatedRecognitions = tfod.getUpdatedRecognitions();
        int foundDucky = foundDuck();
        if (!updatedRecognitions.get(foundDucky).equals(0)) {
            return true;
        }
        else if (tries <= 0) {
            return false;
        }
        else {
//            sleep(250); use sleep if you want to keep track of how much time was lost in searching
            return findDuck(tries-1);
        }
    }

    public int foundDuck() {
        int index = 0;
        for (Recognition recognition : updatedRecognitions) {
            String currentLabel = recognition.getLabel();
            if (currentLabel.equals("Duck")) {
                return index;
            }
            index += 1;
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
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");

        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        cameraServo.setDirection(Servo.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);

        // you set your starting position (try to keep is to a 0 or 1 for later code to actually work)
        cameraServo.setPosition(0);
        // sleep does in milliseconds - 1 second = 1000 seconds
        // test out how fast a servo moves to know the actual amount of time you should give it to move
        sleep(servoMovementDuration);

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
            // 1 zoom mean you use the whole camera(play around with it to find a good zoom)
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        /*
         adjust zoom till it sees 1 possible place where the ducky can be but
         has a big enough range that it won't need the most perfect placement to see the spot
         */
        tfod.setZoom(2.7, 16.0/9.0);

        /*
            depending on what was your starting position of your servo, then that decides what spot
            you just recognized (try to make your starting position one of the side markers
            as if you make it the middle marker then the movement won't be as linear)
        */
        boolean wasThereDuck = findDuck(10);
        if (wasThereDuck) {
            leftBackDrive.setVelocity(3500);
            sleep(1000);
        }
        else {
            cameraServo.setPosition(servoMovement);
            sleep(servoMovementDuration);
            wasThereDuck = findDuck(10);
            if (wasThereDuck) {
                rightBackDrive.setVelocity(3500);
                sleep(1000);
            }

            else {
                /*
                if we already checked two spots then it must be in the last spot if the code worked
                 */
                leftFrontDrive.setVelocity(3500);
                sleep(1000);
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
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}

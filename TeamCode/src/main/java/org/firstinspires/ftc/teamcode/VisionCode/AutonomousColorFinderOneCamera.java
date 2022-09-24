package org.firstinspires.ftc.teamcode.VisionCode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import java.util.List;
import java.util.concurrent.TimeUnit;

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
@Autonomous(name = "AutonomousColorFinderOneCamera", group = "Linear Opmode")
public class AutonomousColorFinderOneCamera extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double globalMovementTimer = 0;
    private double servoMovementDuration = 2;
    // a wheel will be moved accordingly depending on where the camera detected the ducky
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private int clipLeft = 280;
    private int clipTop = 264;
    private int clipRight = 260;
    private int clipBottom = 160;
    private int clipWidth = clipRight;
    private int clipHeight = clipBottom;
    private int wantedColorPixelCount = 0;
    private int thresholdColorPixCount = 10;
    private Bitmap currentBitMap;
    private final String LABEL_QUAD = "Quad";
    private int[][] countColorPixels;
    private int numColorPixels = 0;
    private int purpleR = 0;
    private int purpleG = 0;
    private int purpleB = 0;

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
            "AYClyiX/////AAABmZUUoZkGTkIoi+LAKCAqGAFdzuhe5c9JRs8t0FPGyKqqQ9L21NYhpMmVbdDpNvJQ+S/hw75tU61XfKig8CFsuF0M2IwvX1amtpdJoBVrEQsPdtz5aQTgv+8cyZr6kJT7qZNk8rpLMING/hN/q9BTPY98ugi9v7eNvM1BDt5mp9pZdbwjTxBhgpjKByqDTYMk8fMERc9/Jun3JH9YirEjApUFg/4TDFDOixu6qbMNR8K38BqXmtdwaH04DUVpSSt49LgPMU3rcVWurOojOQ0EQdVOTTCPs/tNyHQCGrXgeAXDg806z0nPibw3ne2wp95lELRegYHGIraNEg7zI2s75sBrIUI1gDm/j8ix3S/jXTzA";

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

    public int isPurple(int pixel) {
        int r = Color.red(pixel);
        int g = Color.green(pixel);
        int b = Color.blue(pixel);

        if (r >= 100) {
            if (b >= 100) {
                if (g >= 0) {
                    return 1;
                }
                return 0;
            }
            return 0;
        }
        return 0;
    }

    public void updateBitmap(Image image) {
        int bufWidth = image.getBufferWidth();
        int bufHeight = image.getBufferHeight();

        int inGrnCnt=0, inNotGrnCnt=0;
        int outGrnCnt=0, outNotGrnCnt=0;

        Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(),
                Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(image.getPixels());
        currentBitMap = bitmap;
    }

    public void updateNumColor() {
        int[][] newColorPixels = new int[clipHeight/8]
                [clipWidth/8];
        for (int row = 0; row < (clipHeight/8); row++) {
            for (int col = 0; col < (clipWidth/8); col++) {
                int pixel = currentBitMap.getPixel(clipLeft+(col*8), clipTop+(row*8));
                newColorPixels[row][col] = isPurple(pixel);
            }
        }
        countColorPixels = newColorPixels;
    }

    public int getNumColorPixels() {
        int count = 0;
        for (int row = 0; row < countColorPixels.length; row++) {
            for (int col = 0; col < countColorPixels[0].length; col++) {
                int val = countColorPixels[row][col];
                if (val == 1) {
                    count++;
                }
            }
        }
        return count;
    }

    public String printableColorMap(int[][] colormap) {
        String finalString = "";
        for (int row = 0; row < colormap.length; row++) {
            for (int col = 0; col < colormap[0].length; col++) {
                finalString += colormap[row][col];
            }
            finalString += "\n";
        }
        return finalString;
    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

//        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
//        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
//        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");
//
//        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
//
//        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);

        // clipping/ focusing on only one part of screen

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
        VuforiaLocalizer.CloseableFrame frame;

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        try {
            frame = vuforia.getFrameQueue().poll(100, TimeUnit.MILLISECONDS);
        } catch (Exception e) {
            frame = null;
        }

        if (frame != null) {
            long numImgs = frame.getNumImages();
            for (int i = 0; i < numImgs; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    updateBitmap(frame.getImage(i));
                    telemetry.addData("bitmap Height: ", currentBitMap.getHeight()); // 720
                    telemetry.addData("bitmap Width: ", currentBitMap.getWidth()); // 1280
                    telemetry.update();
                    //sleep(3000);
                    updateNumColor();
                }
            }
        }
        numColorPixels = getNumColorPixels();

        int midPixel = currentBitMap.getPixel(clipLeft+(clipWidth/2), clipTop+(clipHeight/2));
        purpleR = Color.red(midPixel);
        purpleG = Color.green(midPixel);
        purpleB = Color.blue(midPixel);

        telemetry.addData("purpleR: ", purpleR); // 148, 140, 123
        telemetry.addData("purpleG: ", purpleG); // 121, 105, 93
        telemetry.addData("purpleB: ", purpleB); // 181, 132, 140
        telemetry.addData("numberOfPurplePixels: ", numColorPixels);
        telemetry.addData("colormap: ", printableColorMap(countColorPixels));

        telemetry.update();

        sleep(20000);
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

        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD);
        tfod.setClippingMargins(clipLeft, clipTop, clipRight, clipBottom);
    }
}




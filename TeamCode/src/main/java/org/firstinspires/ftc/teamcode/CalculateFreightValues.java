package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class CalculateFreightValues {
    public CalculateFreightValues() {

    }

    public Recognition findBiggestFreight(List<Recognition> updatedRecognitions) {
        int currentBiggest = 0;
        float biggestRecArea = 0;
        if (updatedRecognitions != null) {
            for (int i = 0; i < updatedRecognitions.size(); i++) {
                Recognition currentRecogntion = updatedRecognitions.get(i);
                float currentRecArea = currentRecogntion.getHeight() * currentRecogntion.getWidth();
                if (currentRecArea >= biggestRecArea) {
                    biggestRecArea = currentRecArea;
                    currentBiggest = i;
                }
            }
        }
        return updatedRecognitions.get(currentBiggest);
    }

    // camera is 1280*720 pixels
    // camera model = logitech hd webcam c310
    //Distance to object(mm) = (f(mm) * real height(mm) * image height(pixels))/ (object height(pixels) * sensor height(mm))
    // f = focal length of camera
    // real height = height of object found
    // image height = resolution of photo
    // object height = height of box around object
    // sensor height = physical size of camera sensor(change with magnification set)
    public double getWidthDistanceToObject(Recognition object) {
        // measurements in mm
        double focalLength = 2;
        double realWidth = 50.8;
        int resolution = 1280;
        double imageWidth = object.getImageWidth();
        double objectWidth = object.getWidth();
        double angleToObject = object.estimateAngleToObject(AngleUnit.DEGREES);
        double sensorHeight = 1;
        return 1;
    }
}

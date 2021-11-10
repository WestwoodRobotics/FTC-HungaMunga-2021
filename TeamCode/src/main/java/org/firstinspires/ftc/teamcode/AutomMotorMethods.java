package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.Math;
import java.util.HashMap;

public class AutomMotorMethods {
    final int WHEELTICKS;
    double robotAngle;
    double robotFrontAngle;
    double[] robotPos = new double[2];


    // constructor the sets the value of WHEELTICKS
    public AutomMotorMethods(int ticks, double robotStartingAngle, double frontAngleOfRobot,
                             double robotYPos, double robotXPos) {
        WHEELTICKS = ticks;
        setRobotAngle(robotStartingAngle);
        setRobotFrontAngle(frontAngleOfRobot);
        robotPos[0] = robotXPos;
        robotPos[1] = robotYPos;
    }

    // return the amount of ticks the wheel has
    public int getWHEELTICKS() {
        return WHEELTICKS;
    }

    public void setRobotAngle(double startingRobotAngle) {
        robotAngle = startingRobotAngle;
    }

    public void setRobotFrontAngle(double frontAngle) {
        robotFrontAngle = frontAngle;
    }

    // returns circumference of mecanum wheels
    public double getCircumference() {
        double circumference = Math.PI*Math.pow(48, 2);
        return circumference;
    }

    // return current angle of robot
    public double getCurrentRobotAngle() {
        return robotAngle;
    }

    public double feetTomm(double feet) {
        double mmDistance = feet*304.8;
        return mmDistance;
    }

    // not used
    public double secondsAnalyzed(double totalSeconds, double excludedSeconds) {
        // subtracting the seconds that are not being counted minus the total seconds
        double analyzedSeconds = totalSeconds - excludedSeconds;
        return analyzedSeconds;
    }

    // return number of ticks the robot moved for given time(seconds) amount
    public int getTicksCrossed(double analyzedSeconds) {
        int numTicks = (int)(analyzedSeconds*560);
        return numTicks;
    }

    /* returns the amount of seconds the robot needs to move for given amount of ticks the
     the robot needs to move
     */
    public double moveForSeconds(int ticks) {
        double secondsAmount = ticks/3500;
        return secondsAmount;
    }

    // returns the distance crossed for given tick amount
    public double distanceCrossed(int ticksCrossed) {
        double circumference = Math.PI*Math.pow(48, 2);
        double distancePerTick = circumference/560;
        // distance is in mm = millimiters
        double distCrossed = ticksCrossed*distancePerTick;
        return distCrossed;
    }

    // returns the tick amount for the given distance
    public int tickForDistance(int distanceInmm) {
        double circumference = Math.PI*Math.pow(48, 2);
        double distancePerTick = circumference/560;
        int distanceToTicks = (int)(distanceInmm/distancePerTick);
        // distance is in mm = millimiters
        return distanceToTicks;
    }

    // converts an angle from 0-355 to a distance to move your wheels
    public int angleToDistance(double angle) {
        double circumference = this.getCircumference();
        double splittingCircIntoFour = circumference/4;
        double distPer360angle = splittingCircIntoFour/90;
        int coverDistance = (int)(distPer360angle*angle);
        return coverDistance;
    }

    // updates angle of robot
    public void updateAngle(double newestAngle) {
        robotAngle = newestAngle;
    }

    /*
    you give it the wheel you want the know the direction it will spin to according
    to the direction the robot will turn to
     */
    public String getWheelSpinDirectionFromWantedRotationDirection(String wantedTurnDirection,
                                                                   String wantedWheel) {
        String leftFrontSpinDirection = "";
        String leftBackSpinDirection = "";
        String rightFrontSpinDirection = "";
        String rightBackSpinDirection = "";
        /* making a dictionary storing a wheel and it's according spin direction
        according to robot spin direction */
        HashMap<String, String> wheelDirections = new HashMap<String, String>();

        if (wantedTurnDirection == "RIGHT") {
            leftFrontSpinDirection = "FORWARDS";
            leftBackSpinDirection = "FORWARDS";
            rightFrontSpinDirection = "BACKWARDS";
            rightBackSpinDirection = "BACKWARDS";
        }
        else if (wantedTurnDirection == "LEFT") {
            leftFrontSpinDirection = "BACKWARDS";
            leftBackSpinDirection = "BACKWARDS";
            rightFrontSpinDirection = "FORWARDS";
            rightBackSpinDirection = "FORWARDS";
        }

        wheelDirections.put("leftFront", leftFrontSpinDirection);
        wheelDirections.put("leftBack", leftBackSpinDirection);
        wheelDirections.put("rightFront", rightFrontSpinDirection);
        wheelDirections.put("rightBack", rightBackSpinDirection);

        String wantedWheelSpinDicrection = wheelDirections.get(wantedWheel);

        return wantedWheelSpinDicrection;
    }

    // returns a number value for an according direction of wheel spin
    public int getVelocityNumFromWheelSpinDirection(String wheelDirection) {
        HashMap<String, Integer> wheelNumDirection = new HashMap<String, Integer>();
        wheelNumDirection.put("FORWARDS", 1);
        wheelNumDirection.put("BACKWARDS", -1);

        int wheelDirectionNumValue = wheelNumDirection.get(wheelDirection);

        return wheelDirectionNumValue;
    }


    public int findOptimumMovementForRobotRotation(double wantedAngle, String wantedWheel) {
        HashMap<String, Integer> wheelDirections = new HashMap<String, Integer>();

        String leftFrontSpinDirectionStringValue = "";
        String leftBackSpinDirectionStringValue = "";
        String rightFrontSpinDirectionStringValue = "";
        String rightBackSpinDirectionStringValue = "";
        int leftFrontSpinDirection = 0;
        int leftBackSpinDirection = 0;
        int rightFrontSpinDirection = 0;
        int rightBackSpinDirection = 0;
        double currentRobotAngle = this.getCurrentRobotAngle();
        double distanceFromAngleMovingLeft = Math.abs(wantedAngle - currentRobotAngle);
        double distanceFromAngleMovingRight = Math.abs((360-currentRobotAngle) + wantedAngle);
        boolean moveLeft = false;
        String moveInDirection = "";
        if (distanceFromAngleMovingRight > distanceFromAngleMovingLeft) {
            moveLeft = true;
            moveInDirection = "LEFT";
            leftFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "leftFront");
            leftBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "leftBack");
            rightFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "rightFront");
            rightBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "rightback");
            leftFrontSpinDirection = getVelocityNumFromWheelSpinDirection
                    (leftFrontSpinDirectionStringValue);
            leftBackSpinDirection = getVelocityNumFromWheelSpinDirection
                    (leftBackSpinDirectionStringValue);
            rightFrontSpinDirection = getVelocityNumFromWheelSpinDirection
                    (rightFrontSpinDirectionStringValue);
            rightBackSpinDirection = getVelocityNumFromWheelSpinDirection
                    (rightBackSpinDirectionStringValue);
        }
        else {
            moveLeft = false;
            moveInDirection = "RIGHT";
            leftFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "leftFront");
            leftBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "leftBack");
            rightFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "rightFront");
            rightBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "rightback");
            leftFrontSpinDirection = getVelocityNumFromWheelSpinDirection
                    (leftFrontSpinDirectionStringValue);
            leftBackSpinDirection = getVelocityNumFromWheelSpinDirection
                    (leftBackSpinDirectionStringValue);
            rightFrontSpinDirection = getVelocityNumFromWheelSpinDirection
                    (rightFrontSpinDirectionStringValue);
            rightBackSpinDirection = getVelocityNumFromWheelSpinDirection
                    (rightBackSpinDirectionStringValue);
        }

        wheelDirections.put("leftFront", leftFrontSpinDirection);
        wheelDirections.put("leftBack", leftBackSpinDirection);
        wheelDirections.put("rightFront", rightFrontSpinDirection);
        wheelDirections.put("rightBack", rightBackSpinDirection);

        int wantedWheelValue = wheelDirections.get(wantedWheel);
        return wantedWheelValue;
    }

    public void updatePos(double movementXmm, double movementYmm) {
        robotPos[0] += movementXmm;
        robotPos[1] += movementYmm;
    }



}

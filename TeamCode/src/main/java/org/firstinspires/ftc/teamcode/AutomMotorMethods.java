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
    final int ROBOTVELOCITY;


    // constructor the sets the value of WHEELTICKS
    public AutomMotorMethods(int ticks, double robotStartingAngle, double frontAngleOfRobot,
                             double robotYPos, double robotXPos, int Robotvelocity) {
        WHEELTICKS = ticks;
        setRobotAngle(robotStartingAngle);
        setRobotFrontAngle(frontAngleOfRobot);
        robotPos[0] = robotXPos;
        robotPos[1] = robotYPos;
        ROBOTVELOCITY = Robotvelocity;
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
        double circumference = 2 * Math.PI * 48;
        return circumference;
    }

    // return current angle of robot
    public double getCurrentRobotAngle() {
        return robotAngle;
    }

    public double feetTomm(double feet) {
        double mmDistance = feet * 304.8;
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
        int numTicks = (int) (analyzedSeconds * 3500);
        return numTicks;
    }

    /* returns the amount of seconds the robot needs to move for given amount of ticks the
     the robot needs to move
     */
    public double moveForSeconds(int ticks, int otherVelocity, boolean useOtherVelocity) {
        if (useOtherVelocity == true) {
            double secondsAmount = ticks / (double) otherVelocity;
            return secondsAmount;
        } else {
            double secondsAmount = ticks / (double) ROBOTVELOCITY;
            return secondsAmount;
        }
    }

    // returns the distance crossed for given tick amount
    public double distanceCrossed(int ticksCrossed) {
        double circumference = 2 * Math.PI * 48;
        double distancePerTick = circumference / 560;
        // distance is in mm = millimiters
        double distCrossed = ticksCrossed * distancePerTick;
        return distCrossed;
    }

    // returns the tick amount for the given distance
    public int tickForDistance(int distanceInmm) {
        double circumference = 2 * Math.PI * 48;
        double distancePerTick = circumference / 560;
        int distanceToTicks = (int) (distanceInmm / distancePerTick);
        // distance is in mm = millimiters
        return distanceToTicks;
    }

    // converts an angle from 0-355 to a distance to move your wheels
    public int angleToDistance(double angle) {
        double circumference = this.getCircumference();
        double splittingCircIntoFour = circumference / 4;
        double distPer360angle = splittingCircIntoFour / 90;
        distPer360angle = circumference/360;
        int coverDistance = (int) (distPer360angle * angle);
        return coverDistance;
    }

    public double distanceForVelocityAndTime(int velocity, double seconds) {
        double distance = velocity*seconds;
        return distance;
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
        } else if (wantedTurnDirection == "LEFT") {
            leftFrontSpinDirection = "BACKWARDS";
            leftBackSpinDirection = "BACKWARDS";
            rightFrontSpinDirection = "FORWARDS";
            rightBackSpinDirection = "FORWARDS";
        }

        wheelDirections.put("leftFront", leftFrontSpinDirection);
        wheelDirections.put("leftBack", leftBackSpinDirection);
        wheelDirections.put("rightFront", rightFrontSpinDirection);
        wheelDirections.put("rightBack", rightBackSpinDirection);

        String wantedWheelSpinDirection = wheelDirections.get(wantedWheel);

        return wantedWheelSpinDirection;
    }

    // returns a number value for an according direction of wheel spin
    public int getVelocityNumFromWheelSpinDirection(String wheelDirection) {
        HashMap<String, Integer> wheelNumDirection = new HashMap<String, Integer>();
        wheelNumDirection.put("FORWARDS", 1);
        wheelNumDirection.put("BACKWARDS", -1);

        Integer wheelDirectionNumValue = wheelNumDirection.get(wheelDirection);
        if (wheelDirection.equals("FORWARDS")) {
            return 1;
        } else if (wheelDirection.equals("BACKWARDS")) {
            return -1;
        }
        else {
            return 0;
        }
    }

    public double findCircleDistanceAccordingToRL(String wantedDirection,
                                                  double angleDistance, double currentRobotAngle) {
        double distanceFirstPart = 0;
        double distanceSecondPart = 0;
        if (wantedDirection.equals("LEFT")) {
            if (angleDistance < 0) {
                distanceFirstPart = 360 - currentRobotAngle;
                distanceSecondPart = Math.abs(angleDistance);
                return distanceFirstPart + distanceSecondPart;
            }
            else {
                return angleDistance;
            }
        }
        else {
            if (angleDistance > 360) {
                distanceFirstPart = currentRobotAngle;
                distanceSecondPart = Math.abs(angleDistance-360);
                return distanceFirstPart + distanceSecondPart;
            }
            else {
                return angleDistance;
            }
        }
    }

    // finding if to move left or right to reach wanted angle
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
        double angleDistanceFromAngleMovingLeft = Math.abs(wantedAngle - currentRobotAngle);
        double angleDistanceLeft = findCircleDistanceAccordingToRL("LEFT",
                angleDistanceFromAngleMovingLeft, currentRobotAngle);
        double angleDistanceFromAngleMovingRight = (360 - currentRobotAngle) + wantedAngle;
        double angleDistanceRight = findCircleDistanceAccordingToRL("RIGHT",
                angleDistanceFromAngleMovingRight, currentRobotAngle);
//        double distanceFromAngleMovingLeft = Math.abs(wantedAngle - currentRobotAngle);
//        double distanceFromAngleMovingRight = Math.abs((360 - currentRobotAngle) + wantedAngle);
        boolean moveLeft = false;
        String moveInDirection = "";
        if (angleDistanceRight > angleDistanceLeft) {
            moveLeft = true;
            moveInDirection = "LEFT";
            leftFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "leftFront");
            leftBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "leftBack");
            rightFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "rightFront");
            rightBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "rightBack");
            leftFrontSpinDirection = getVelocityNumFromWheelSpinDirection
                    (leftFrontSpinDirectionStringValue);
            leftBackSpinDirection = getVelocityNumFromWheelSpinDirection
                    (leftBackSpinDirectionStringValue);
            rightFrontSpinDirection = getVelocityNumFromWheelSpinDirection
                    (rightFrontSpinDirectionStringValue);
            rightBackSpinDirection = getVelocityNumFromWheelSpinDirection
                    (rightBackSpinDirectionStringValue);
        } else {
            moveLeft = false;
            moveInDirection = "RIGHT";
            leftFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "leftFront");
            leftBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "leftBack");
            rightFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "rightFront");
            rightBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
                    (moveInDirection, "rightBack");
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

    public double doesItSurpassTicks(int ticksTraversed, int wantedTicks,
                                     double durationOfCurrentStage, int ticksTraversedByCurrentStage,
                                     int currentStageVelocity) {
        int tickDifference = 0;
        int newNumTicksOfStage = 0;
        boolean doesItSurpassWantedTicks = false;
        double newStageDuration = 0;
        if (ticksTraversed >= wantedTicks) {
            doesItSurpassWantedTicks = true;
            tickDifference = ticksTraversed - wantedTicks;
            newNumTicksOfStage = ticksTraversedByCurrentStage - tickDifference;
            newStageDuration = this.moveForSeconds(newNumTicksOfStage, currentStageVelocity, true);
        }
        if (doesItSurpassWantedTicks == true) {
            return newStageDuration;
        } else {
            return durationOfCurrentStage;
        }
    }

    public HashMap<String, Double> stageVelocitySetUp(int stageVelocity, double secondsAmount,
                                                      int totalTicksTraveled, int wantedTicks,
                                                      double currentStagesused) {
        HashMap<String, Double> infoHolder = new HashMap<String, Double>();
        boolean reachedWantedTicks = false;
        int StageTicksTraveled = (int) (stageVelocity * secondsAmount);
        totalTicksTraveled += StageTicksTraveled;
        double doesitSurpassWantedTicks = doesItSurpassTicks(totalTicksTraveled, wantedTicks,
                secondsAmount, StageTicksTraveled, stageVelocity);
        if (doesitSurpassWantedTicks == secondsAmount) {
            reachedWantedTicks = false;
        } else {
            reachedWantedTicks = true;
        }
        double numStagesUsed = currentStagesused + 1;
        infoHolder.put("stageDuration", doesitSurpassWantedTicks);
        infoHolder.put("stageVelocity", (double) stageVelocity);
        infoHolder.put("numStagesUsed", numStagesUsed);
        return infoHolder;
    }

    public HashMap<String, Double> generateMotionProfile(int wantedTicks, double velocityGoal) {
        HashMap<String, Double> StageInfoHolder = new HashMap<String, Double>();
//        int velocityAddUp = (int) (velocityGoal) / 5;
        int velocityAddUp = (int) (velocityGoal) / 20;
        double halfSecond = .1;
        int totalTicksTraveled = 0;
        boolean reachedWantedTicks = false;
        double didItSurpassWantedTicksStage1;
        double didItSurpassWantedTicksStage2;
        double didItSurpassWantedTicksStage3;
        double didItSurpassWantedTicksStage4;
        double didItSurpassWantedTicksStage5;
        double didItSurpassWantedTicksStage6;
        double didItSurpassWantedTicksStage7;
        double didItSurpassWantedTicksStage8;
        double didItSurpassWantedTicksStage9;
        double didItSurpassWantedTicksStage10;
        double didItSurpassWantedTicksStage11;
        double didItSurpassWantedTicksStage12;
        double didItSurpassWantedTicksStage13;
        double didItSurpassWantedTicksStage14;
        double didItSurpassWantedTicksStage15;
        double didItSurpassWantedTicksStage16;
        double didItSurpassWantedTicksStage17;
        double didItSurpassWantedTicksStage18;
        double didItSurpassWantedTicksStage19;
        double didItSurpassWantedTicksStage20;
        int numStagesUsed = 0;
        int ticksLeftForStage20 = 0;
        double stage20Seconds = 0;

        // stage1 set-up
        int Stage1Velocity = velocityAddUp * 1;
        stageVelocitySetUp(Stage1Velocity, halfSecond, totalTicksTraveled, wantedTicks, numStagesUsed);
        int Stage1TicksTraveled = (int) (Stage1Velocity * halfSecond);
        totalTicksTraveled += Stage1TicksTraveled;
        didItSurpassWantedTicksStage1 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                Stage1TicksTraveled, Stage1Velocity);
        if (didItSurpassWantedTicksStage1 == halfSecond) {
            reachedWantedTicks = false;
        } else {
            reachedWantedTicks = true;
        }
        StageInfoHolder.put("stage1Duration", didItSurpassWantedTicksStage1);
        StageInfoHolder.put("stage1Velocity", (double) Stage1Velocity);
        StageInfoHolder.put("stage1Ticks", (double)Stage1TicksTraveled);
        numStagesUsed += 1;

        if (reachedWantedTicks == false) {
            //stage2 set-up
            int Stage2Velocity = velocityAddUp * 2;
            int Stage2TicksTraveled = (int) (Stage2Velocity * halfSecond);
            totalTicksTraveled += Stage2TicksTraveled;
            didItSurpassWantedTicksStage2 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                    Stage2TicksTraveled, Stage2Velocity);
            if (didItSurpassWantedTicksStage2 == halfSecond) {
                reachedWantedTicks = false;
            } else {
                reachedWantedTicks = true;
            }
            StageInfoHolder.put("stage2Duration", didItSurpassWantedTicksStage2);
            StageInfoHolder.put("stage2Velocity", (double) Stage2Velocity);
            StageInfoHolder.put("stage2Ticks", (double)Stage2TicksTraveled);
            numStagesUsed += 1;

            if (reachedWantedTicks == false) {
                //stage3 set-up
                int Stage3Velocity = velocityAddUp * 3;
                int Stage3TicksTraveled = (int) (Stage3Velocity * halfSecond);
                totalTicksTraveled += Stage3TicksTraveled;
                didItSurpassWantedTicksStage3 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                        Stage3TicksTraveled, Stage3Velocity);
                if (didItSurpassWantedTicksStage3 == halfSecond) {
                    reachedWantedTicks = false;
                } else {
                    reachedWantedTicks = true;
                }
                StageInfoHolder.put("stage3Duration", didItSurpassWantedTicksStage3);
                StageInfoHolder.put("stage3Velocity", (double) Stage3Velocity);
                StageInfoHolder.put("stage3Ticks", (double)Stage3TicksTraveled);
                numStagesUsed += 1;

                if (reachedWantedTicks == false) {
                    //stage4 set-up
                    int Stage4Velocity = velocityAddUp * 4;
                    int Stage4TicksTraveled = (int) (Stage4Velocity * halfSecond);
                    totalTicksTraveled += Stage4TicksTraveled;
                    didItSurpassWantedTicksStage4 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                            Stage4TicksTraveled, Stage4Velocity);
                    if (didItSurpassWantedTicksStage4 == halfSecond) {
                        reachedWantedTicks = false;
                    } else {
                        reachedWantedTicks = true;
                    }
                    StageInfoHolder.put("stage4Duration", didItSurpassWantedTicksStage4);
                    StageInfoHolder.put("stage4Velocity", (double) Stage4Velocity);
                    StageInfoHolder.put("stage4Ticks", (double)Stage4TicksTraveled);
                    numStagesUsed += 1;

//                    if (reachedWantedTicks == false) {
//                        //stage5 set-up
//                        ticksLeftForStage5 = wantedTicks - totalTicksTraveled;
//                        int Stage5Velocity = velocityAddUp * 5;
//                        stage5Seconds = this.moveForSeconds(ticksLeftForStage5, Stage5Velocity,
//                                true);
//                        StageInfoHolder.put("stage5Duration", stage5Seconds);
//                        StageInfoHolder.put("stage5Velocity", (double)Stage5Velocity);
//
//                    }
                    if (reachedWantedTicks == false) {
                        //stage5 set-up
                        int Stage5Velocity = velocityAddUp * 5;
                        int Stage5TicksTraveled = (int) (Stage5Velocity * halfSecond);
                        totalTicksTraveled += Stage5TicksTraveled;
                        didItSurpassWantedTicksStage5 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                Stage5TicksTraveled, Stage5Velocity);
                        if (didItSurpassWantedTicksStage5 == halfSecond) {
                            reachedWantedTicks = false;
                        } else {
                            reachedWantedTicks = true;
                        }
                        StageInfoHolder.put("stage5Duration", didItSurpassWantedTicksStage5);
                        StageInfoHolder.put("stage5Velocity", (double) Stage5Velocity);
                        StageInfoHolder.put("stage5Ticks", (double)Stage5TicksTraveled);
                        numStagesUsed += 1;

                        if (reachedWantedTicks == false) {
                            //stage6 set-up
                            int Stage6Velocity = velocityAddUp * 6;
                            int Stage6TicksTraveled = (int) (Stage6Velocity * halfSecond);
                            totalTicksTraveled += Stage6TicksTraveled;
                            didItSurpassWantedTicksStage6 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                    Stage6TicksTraveled, Stage6Velocity);
                            if (didItSurpassWantedTicksStage6 == halfSecond) {
                                reachedWantedTicks = false;
                            } else {
                                reachedWantedTicks = true;
                            }
                            StageInfoHolder.put("stage6Duration", didItSurpassWantedTicksStage6);
                            StageInfoHolder.put("stage6Velocity", (double) Stage6Velocity);
                            StageInfoHolder.put("stage6Ticks", (double)Stage6TicksTraveled);
                            numStagesUsed += 1;

                            if (reachedWantedTicks == false) {
                                //stage7 set-up
                                int Stage7Velocity = velocityAddUp * 7;
                                int Stage7TicksTraveled = (int) (Stage7Velocity * halfSecond);
                                totalTicksTraveled += Stage7TicksTraveled;
                                didItSurpassWantedTicksStage7 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                        Stage7TicksTraveled, Stage7Velocity);
                                if (didItSurpassWantedTicksStage7 == halfSecond) {
                                    reachedWantedTicks = false;
                                } else {
                                    reachedWantedTicks = true;
                                }
                                StageInfoHolder.put("stage7Duration", didItSurpassWantedTicksStage7);
                                StageInfoHolder.put("stage7Velocity", (double) Stage7Velocity);
                                StageInfoHolder.put("stage7Ticks", (double)Stage7TicksTraveled);
                                numStagesUsed += 1;

                                if (reachedWantedTicks == false) {
                                    //stage8 set-up
                                    int Stage8Velocity = velocityAddUp * 8;
                                    int Stage8TicksTraveled = (int) (Stage8Velocity * halfSecond);
                                    totalTicksTraveled += Stage8TicksTraveled;
                                    didItSurpassWantedTicksStage8 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                            Stage8TicksTraveled, Stage8Velocity);
                                    if (didItSurpassWantedTicksStage8 == halfSecond) {
                                        reachedWantedTicks = false;
                                    } else {
                                        reachedWantedTicks = true;
                                    }
                                    StageInfoHolder.put("stage8Duration", didItSurpassWantedTicksStage8);
                                    StageInfoHolder.put("stage8Velocity", (double) Stage8Velocity);
                                    StageInfoHolder.put("stage8Ticks", (double)Stage8TicksTraveled);
                                    numStagesUsed += 1;

                                    if (reachedWantedTicks == false) {
                                        //stage9 set-up
                                        int Stage9Velocity = velocityAddUp * 9;
                                        int Stage9TicksTraveled = (int) (Stage9Velocity * halfSecond);
                                        totalTicksTraveled += Stage9TicksTraveled;
                                        didItSurpassWantedTicksStage9 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                Stage9TicksTraveled, Stage9Velocity);
                                        if (didItSurpassWantedTicksStage9 == halfSecond) {
                                            reachedWantedTicks = false;
                                        } else {
                                            reachedWantedTicks = true;
                                        }
                                        StageInfoHolder.put("stage9Duration", didItSurpassWantedTicksStage9);
                                        StageInfoHolder.put("stage9Velocity", (double) Stage9Velocity);
                                        StageInfoHolder.put("stage9Ticks", (double)Stage9TicksTraveled);
                                        numStagesUsed += 1;

//                                        if (reachedWantedTicks == false) {
//                                            //stage10 set-up
//                                            ticksLeftForStage10 = wantedTicks - totalTicksTraveled;
//                                            int Stage10Velocity = velocityAddUp * 5;
//                                            stage10Seconds = this.moveForSeconds(ticksLeftForStage10, Stage10Velocity,
//                                                    true);
//                                            StageInfoHolder.put("stage10Duration", stage10Seconds);
//                                            StageInfoHolder.put("stage10Velocity", (double) Stage10Velocity);
//                                            numStagesUsed += 1;

                                        if (reachedWantedTicks == false) {
                                            //stage10 set-up
                                            int Stage10Velocity = velocityAddUp * 10;
                                            int Stage10TicksTraveled = (int) (Stage10Velocity * halfSecond);
                                            totalTicksTraveled += Stage10TicksTraveled;
                                            didItSurpassWantedTicksStage10 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                    Stage10TicksTraveled, Stage10Velocity);
                                            if (didItSurpassWantedTicksStage10 == halfSecond) {
                                                reachedWantedTicks = false;
                                            } else {
                                                reachedWantedTicks = true;
                                            }
                                            StageInfoHolder.put("stage10Duration", didItSurpassWantedTicksStage10);
                                            StageInfoHolder.put("stage10Velocity", (double) Stage10Velocity);
                                            StageInfoHolder.put("stage10Ticks", (double)Stage10TicksTraveled);
                                            numStagesUsed += 1;

                                            if (reachedWantedTicks == false) {
                                                //stage11 set-up
                                                int Stage11Velocity = velocityAddUp * 11;
                                                int Stage11TicksTraveled = (int) (Stage11Velocity * halfSecond);
                                                totalTicksTraveled += Stage11TicksTraveled;
                                                didItSurpassWantedTicksStage11 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                        Stage11TicksTraveled, Stage11Velocity);
                                                if (didItSurpassWantedTicksStage11 == halfSecond) {
                                                    reachedWantedTicks = false;
                                                } else {
                                                    reachedWantedTicks = true;
                                                }
                                                StageInfoHolder.put("stage11Duration", didItSurpassWantedTicksStage11);
                                                StageInfoHolder.put("stage11Velocity", (double) Stage11Velocity);
                                                StageInfoHolder.put("stage11Ticks", (double)Stage11TicksTraveled);
                                                numStagesUsed += 1;

                                                if (reachedWantedTicks == false) {
                                                    //stage12 set-up
                                                    int Stage12Velocity = velocityAddUp * 12;
                                                    int Stage12TicksTraveled = (int) (Stage12Velocity * halfSecond);
                                                    totalTicksTraveled += Stage12TicksTraveled;
                                                    didItSurpassWantedTicksStage12 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                            Stage12TicksTraveled, Stage12Velocity);
                                                    if (didItSurpassWantedTicksStage11 == halfSecond) {
                                                        reachedWantedTicks = false;
                                                    } else {
                                                        reachedWantedTicks = true;
                                                    }
                                                    StageInfoHolder.put("stage12Duration", didItSurpassWantedTicksStage12);
                                                    StageInfoHolder.put("stage12Velocity", (double) Stage12Velocity);
                                                    StageInfoHolder.put("stage12Ticks", (double)Stage12TicksTraveled);
                                                    numStagesUsed += 1;

                                                    if (reachedWantedTicks == false) {
                                                        //stage13 set-up
                                                        int Stage13Velocity = velocityAddUp * 13;
                                                        int Stage13TicksTraveled = (int) (Stage13Velocity * halfSecond);
                                                        totalTicksTraveled += Stage13TicksTraveled;
                                                        didItSurpassWantedTicksStage13 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                                Stage13TicksTraveled, Stage13Velocity);
                                                        if (didItSurpassWantedTicksStage13 == halfSecond) {
                                                            reachedWantedTicks = false;
                                                        } else {
                                                            reachedWantedTicks = true;
                                                        }
                                                        StageInfoHolder.put("stage13Duration", didItSurpassWantedTicksStage13);
                                                        StageInfoHolder.put("stage13Velocity", (double) Stage13Velocity);
                                                        StageInfoHolder.put("stage13Ticks", (double)Stage13TicksTraveled);
                                                        numStagesUsed += 1;

                                                        if (reachedWantedTicks == false) {
                                                            //stage14 set-up
                                                            int Stage14Velocity = velocityAddUp * 14;
                                                            int Stage14TicksTraveled = (int) (Stage14Velocity * halfSecond);
                                                            totalTicksTraveled += Stage14TicksTraveled;
                                                            didItSurpassWantedTicksStage14 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                                    Stage14TicksTraveled, Stage14Velocity);
                                                            if (didItSurpassWantedTicksStage14 == halfSecond) {
                                                                reachedWantedTicks = false;
                                                            } else {
                                                                reachedWantedTicks = true;
                                                            }
                                                            StageInfoHolder.put("stage14Duration", didItSurpassWantedTicksStage14);
                                                            StageInfoHolder.put("stage14Velocity", (double) Stage14Velocity);
                                                            StageInfoHolder.put("stage14Ticks", (double)Stage14TicksTraveled);
                                                            numStagesUsed += 1;

                                                            if (reachedWantedTicks == false) {
                                                                //stage15 set-up
                                                                int Stage15Velocity = velocityAddUp * 15;
                                                                int Stage15TicksTraveled = (int) (Stage15Velocity * halfSecond);
                                                                totalTicksTraveled += Stage15TicksTraveled;
                                                                didItSurpassWantedTicksStage15 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                                        Stage15TicksTraveled, Stage15Velocity);
                                                                if (didItSurpassWantedTicksStage15 == halfSecond) {
                                                                    reachedWantedTicks = false;
                                                                } else {
                                                                    reachedWantedTicks = true;
                                                                }
                                                                StageInfoHolder.put("stage15Duration", didItSurpassWantedTicksStage15);
                                                                StageInfoHolder.put("stage15Velocity", (double) Stage15Velocity);
                                                                StageInfoHolder.put("stage15Ticks", (double)Stage15TicksTraveled);
                                                                numStagesUsed += 1;

                                                                if (reachedWantedTicks == false) {
                                                                    //stage16 set-up
                                                                    int Stage16Velocity = velocityAddUp * 16;
                                                                    int Stage16TicksTraveled = (int) (Stage16Velocity * halfSecond);
                                                                    totalTicksTraveled += Stage16TicksTraveled;
                                                                    didItSurpassWantedTicksStage16 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                                            Stage16TicksTraveled, Stage16Velocity);
                                                                    if (didItSurpassWantedTicksStage16 == halfSecond) {
                                                                        reachedWantedTicks = false;
                                                                    } else {
                                                                        reachedWantedTicks = true;
                                                                    }
                                                                    StageInfoHolder.put("stage16Duration", didItSurpassWantedTicksStage16);
                                                                    StageInfoHolder.put("stage16Velocity", (double) Stage16Velocity);
                                                                    StageInfoHolder.put("stage16Ticks", (double)Stage16TicksTraveled);
                                                                    numStagesUsed += 1;

                                                                    if (reachedWantedTicks == false) {
                                                                        //stage17 set-up
                                                                        int Stage17Velocity = velocityAddUp * 17;
                                                                        int Stage17TicksTraveled = (int) (Stage17Velocity * halfSecond);
                                                                        totalTicksTraveled += Stage17TicksTraveled;
                                                                        didItSurpassWantedTicksStage17 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                                                Stage17TicksTraveled, Stage17Velocity);
                                                                        if (didItSurpassWantedTicksStage17 == halfSecond) {
                                                                            reachedWantedTicks = false;
                                                                        } else {
                                                                            reachedWantedTicks = true;
                                                                        }
                                                                        StageInfoHolder.put("stage17Duration", didItSurpassWantedTicksStage17);
                                                                        StageInfoHolder.put("stage17Velocity", (double) Stage17Velocity);
                                                                        StageInfoHolder.put("stage17Ticks", (double)Stage17TicksTraveled);
                                                                        numStagesUsed += 1;

                                                                        if (reachedWantedTicks == false) {
                                                                            //stage18 set-up
                                                                            int Stage18Velocity = velocityAddUp * 18;
                                                                            int Stage18TicksTraveled = (int) (Stage18Velocity * halfSecond);
                                                                            totalTicksTraveled += Stage18TicksTraveled;
                                                                            didItSurpassWantedTicksStage18 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                                                    Stage18TicksTraveled, Stage18Velocity);
                                                                            if (didItSurpassWantedTicksStage18 == halfSecond) {
                                                                                reachedWantedTicks = false;
                                                                            } else {
                                                                                reachedWantedTicks = true;
                                                                            }
                                                                            StageInfoHolder.put("stage18Duration", didItSurpassWantedTicksStage18);
                                                                            StageInfoHolder.put("stage18Velocity", (double) Stage18Velocity);
                                                                            StageInfoHolder.put("stage18Ticks", (double)Stage18TicksTraveled);
                                                                            numStagesUsed += 1;

                                                                            if (reachedWantedTicks == false) {
                                                                                //stage19 set-up
                                                                                int Stage19Velocity = velocityAddUp * 19;
                                                                                int Stage19TicksTraveled = (int) (Stage19Velocity * halfSecond);
                                                                                totalTicksTraveled += Stage19TicksTraveled;
                                                                                didItSurpassWantedTicksStage19 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
                                                                                        Stage19TicksTraveled, Stage19Velocity);
                                                                                if (didItSurpassWantedTicksStage19 == halfSecond) {
                                                                                    reachedWantedTicks = false;
                                                                                } else {
                                                                                    reachedWantedTicks = true;
                                                                                }
                                                                                StageInfoHolder.put("stage19Duration", didItSurpassWantedTicksStage19);
                                                                                StageInfoHolder.put("stage19Velocity", (double) Stage19Velocity);
                                                                                StageInfoHolder.put("stage19Ticks", (double)Stage19TicksTraveled);
                                                                                numStagesUsed += 1;

                                                                                if (reachedWantedTicks == false) {
                                                                                    //stage20 set-up
                                                                                    ticksLeftForStage20 = wantedTicks - totalTicksTraveled;
                                                                                    int Stage20Velocity = velocityAddUp * 20;
                                                                                    stage20Seconds = this.moveForSeconds(ticksLeftForStage20, Stage20Velocity,
                                                                                            true);
                                                                                    StageInfoHolder.put("stage20Duration", stage20Seconds);
                                                                                    StageInfoHolder.put("stage20Velocity", (double) Stage20Velocity);
                                                                                    StageInfoHolder.put("stage20Ticks", (double)ticksLeftForStage20);
                                                                                    numStagesUsed += 1;
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        StageInfoHolder.put("numStages", (double) numStagesUsed);

        return StageInfoHolder;
    }

}
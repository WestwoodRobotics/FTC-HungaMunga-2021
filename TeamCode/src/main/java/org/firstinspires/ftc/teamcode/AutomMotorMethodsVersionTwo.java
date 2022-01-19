//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import java.lang.Math;
//import java.util.HashMap;
//
//public class AutomMotorMethodsVersionTwo {
//    final int WHEELTICKS;
//    double robotAngle;
//    double robotFrontAngle;
//    double[] robotPos = new double[2];
//    final int ROBOTVELOCITY;
//
//
//    // constructor the sets the value of WHEELTICKS
//    public AutomMotorMethods(int ticks, double robotStartingAngle, double frontAngleOfRobot,
//                             double robotYPos, double robotXPos, int Robotvelocity) {
//        WHEELTICKS = ticks;
//        setRobotAngle(robotStartingAngle);
//        setRobotFrontAngle(frontAngleOfRobot);
//        robotPos[0] = robotXPos;
//        robotPos[1] = robotYPos;
//        ROBOTVELOCITY = Robotvelocity;
//    }
//
//    // return the amount of ticks the wheel has
//    public int getWHEELTICKS() {
//        return WHEELTICKS;
//    }
//
//    public void setRobotAngle(double startingRobotAngle) {
//        robotAngle = startingRobotAngle;
//    }
//
//    public void setRobotFrontAngle(double frontAngle) {
//        robotFrontAngle = frontAngle;
//    }
//
//    // returns circumference of mecanum wheels
//    public double getCircumference() {
//        double circumference = Math.PI * Math.pow(48, 2);
//        return circumference;
//    }
//
//    // return current angle of robot
//    public double getCurrentRobotAngle() {
//        return robotAngle;
//    }
//
//    public double feetTomm(double feet) {
//        double mmDistance = feet * 304.8;
//        return mmDistance;
//    }
//
//    // not used
//    public double secondsAnalyzed(double totalSeconds, double excludedSeconds) {
//        // subtracting the seconds that are not being counted minus the total seconds
//        double analyzedSeconds = totalSeconds - excludedSeconds;
//        return analyzedSeconds;
//    }
//
//    // return number of ticks the robot moved for given time(seconds) amount
//    public int getTicksCrossed(double analyzedSeconds) {
//        int numTicks = (int) (analyzedSeconds * 3500);
//        return numTicks;
//    }
//
//    /* returns the amount of seconds the robot needs to move for given amount of ticks the
//     the robot needs to move
//     */
//    public double moveForSeconds(int ticks, int otherVelocity, boolean useOtherVelocity) {
//        if (useOtherVelocity == true) {
//            double secondsAmount = ticks / (double) otherVelocity;
//            return secondsAmount;
//        } else {
//            double secondsAmount = ticks / (double) ROBOTVELOCITY;
//            return secondsAmount;
//        }
//    }
//
//    // returns the distance crossed for given tick amount
//    public double distanceCrossed(int ticksCrossed) {
//        double circumference = Math.PI * Math.pow(48, 2);
//        double distancePerTick = circumference / 560;
//        // distance is in mm = millimiters
//        double distCrossed = ticksCrossed * distancePerTick;
//        return distCrossed;
//    }
//
//    // returns the tick amount for the given distance
//    public int tickForDistance(int distanceInmm) {
//        double circumference = 2 * Math.PI * 48;
//        double distancePerTick = circumference / 560;
//        int distanceToTicks = (int) (distanceInmm / distancePerTick);
//        // distance is in mm = millimiters
//        return distanceToTicks;
//    }
//
//    // converts an angle from 0-355 to a distance to move your wheels
//    public int angleToDistance(double angle) {
//        double circumference = this.getCircumference();
//        double splittingCircIntoFour = circumference / 4;
//        double distPer360angle = splittingCircIntoFour / 90;
//        int coverDistance = (int) (distPer360angle * angle);
//        return coverDistance;
//    }
//
//    // updates angle of robot
//    public void updateAngle(double newestAngle) {
//        robotAngle = newestAngle;
//    }
//
//    /*
//    you give it the wheel you want the know the direction it will spin to according
//    to the direction the robot will turn to
//     */
//    public String getWheelSpinDirectionFromWantedRotationDirection(String wantedTurnDirection,
//                                                                   String wantedWheel) {
//        String leftFrontSpinDirection = "";
//        String leftBackSpinDirection = "";
//        String rightFrontSpinDirection = "";
//        String rightBackSpinDirection = "";
//        /* making a dictionary storing a wheel and it's according spin direction
//        according to robot spin direction */
//        HashMap<String, String> wheelDirections = new HashMap<String, String>();
//
//        if (wantedTurnDirection == "RIGHT") {
//            leftFrontSpinDirection = "FORWARDS";
//            leftBackSpinDirection = "FORWARDS";
//            rightFrontSpinDirection = "BACKWARDS";
//            rightBackSpinDirection = "BACKWARDS";
//        } else if (wantedTurnDirection == "LEFT") {
//            leftFrontSpinDirection = "BACKWARDS";
//            leftBackSpinDirection = "BACKWARDS";
//            rightFrontSpinDirection = "FORWARDS";
//            rightBackSpinDirection = "FORWARDS";
//        }
//
//        wheelDirections.put("leftFront", leftFrontSpinDirection);
//        wheelDirections.put("leftBack", leftBackSpinDirection);
//        wheelDirections.put("rightFront", rightFrontSpinDirection);
//        wheelDirections.put("rightBack", rightBackSpinDirection);
//
//        String wantedWheelSpinDicrection = wheelDirections.get(wantedWheel);
//
//        return wantedWheelSpinDicrection;
//    }
//
//    // returns a number value for an according direction of wheel spin
//    public int getVelocityNumFromWheelSpinDirection(String wheelDirection) {
//        HashMap<String, Integer> wheelNumDirection = new HashMap<String, Integer>();
//        wheelNumDirection.put("FORWARDS", 1);
//        wheelNumDirection.put("BACKWARDS", -1);
//
//        int wheelDirectionNumValue = wheelNumDirection.get(wheelDirection);
//
//        return wheelDirectionNumValue;
//    }
//
//
//    public int findOptimumMovementForRobotRotation(double wantedAngle, String wantedWheel) {
//        HashMap<String, Integer> wheelDirections = new HashMap<String, Integer>();
//
//        String leftFrontSpinDirectionStringValue = "";
//        String leftBackSpinDirectionStringValue = "";
//        String rightFrontSpinDirectionStringValue = "";
//        String rightBackSpinDirectionStringValue = "";
//        int leftFrontSpinDirection = 0;
//        int leftBackSpinDirection = 0;
//        int rightFrontSpinDirection = 0;
//        int rightBackSpinDirection = 0;
//        double currentRobotAngle = this.getCurrentRobotAngle();
//        double distanceFromAngleMovingLeft = Math.abs(wantedAngle - currentRobotAngle);
//        double distanceFromAngleMovingRight = Math.abs((360 - currentRobotAngle) + wantedAngle);
//        boolean moveLeft = false;
//        String moveInDirection = "";
//        if (distanceFromAngleMovingRight > distanceFromAngleMovingLeft) {
//            moveLeft = true;
//            moveInDirection = "LEFT";
//            leftFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
//                    (moveInDirection, "leftFront");
//            leftBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
//                    (moveInDirection, "leftBack");
//            rightFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
//                    (moveInDirection, "rightFront");
//            rightBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
//                    (moveInDirection, "rightback");
//            leftFrontSpinDirection = getVelocityNumFromWheelSpinDirection
//                    (leftFrontSpinDirectionStringValue);
//            leftBackSpinDirection = getVelocityNumFromWheelSpinDirection
//                    (leftBackSpinDirectionStringValue);
//            rightFrontSpinDirection = getVelocityNumFromWheelSpinDirection
//                    (rightFrontSpinDirectionStringValue);
//            rightBackSpinDirection = getVelocityNumFromWheelSpinDirection
//                    (rightBackSpinDirectionStringValue);
//        } else {
//            moveLeft = false;
//            moveInDirection = "RIGHT";
//            leftFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
//                    (moveInDirection, "leftFront");
//            leftBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
//                    (moveInDirection, "leftBack");
//            rightFrontSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
//                    (moveInDirection, "rightFront");
//            rightBackSpinDirectionStringValue = getWheelSpinDirectionFromWantedRotationDirection
//                    (moveInDirection, "rightback");
//            leftFrontSpinDirection = getVelocityNumFromWheelSpinDirection
//                    (leftFrontSpinDirectionStringValue);
//            leftBackSpinDirection = getVelocityNumFromWheelSpinDirection
//                    (leftBackSpinDirectionStringValue);
//            rightFrontSpinDirection = getVelocityNumFromWheelSpinDirection
//                    (rightFrontSpinDirectionStringValue);
//            rightBackSpinDirection = getVelocityNumFromWheelSpinDirection
//                    (rightBackSpinDirectionStringValue);
//        }
//
//        wheelDirections.put("leftFront", leftFrontSpinDirection);
//        wheelDirections.put("leftBack", leftBackSpinDirection);
//        wheelDirections.put("rightFront", rightFrontSpinDirection);
//        wheelDirections.put("rightBack", rightBackSpinDirection);
//
//        int wantedWheelValue = wheelDirections.get(wantedWheel);
//        return wantedWheelValue;
//    }
//
//    public void updatePos(double movementXmm, double movementYmm) {
//        robotPos[0] += movementXmm;
//        robotPos[1] += movementYmm;
//    }
//
//    public double doesItSurpassTicks(int ticksTraversed, int wantedTicks,
//                                     double durationOfCurrentStage, int ticksTraversedByCurrentStage,
//                                     int currentStageVelocity) {
//        int tickDifference = 0;
//        int newNumTicksOfStage = 0;
//        boolean doesItSurpassWantedTicks = false;
//        double newStageDuration = 0;
//        if (ticksTraversed >= wantedTicks) {
//            doesItSurpassWantedTicks = true;
//            tickDifference = ticksTraversed - wantedTicks;
//            newNumTicksOfStage = ticksTraversedByCurrentStage - tickDifference;
//            newStageDuration = this.moveForSeconds(newNumTicksOfStage, currentStageVelocity, true);
//        }
//        if (doesItSurpassWantedTicks == true) {
//            return newStageDuration;
//        } else {
//            return durationOfCurrentStage;
//        }
//    }
//
//    public HashMap<String, Double> generateMotionProfile(int wantedTicks, double velocityGoal) {
//        HashMap<String, Double> StageInfoHolder = new HashMap<String, Double>();
////        int velocityAddUp = (int) (velocityGoal) / 5;
//        int velocityAddUp = (int) (velocityGoal) / 10;
//        double halfSecond = .3;
//        int totalTicksTraveled = 0;
//        boolean reachedWantedTicks = false;
//        double didItSurpassWantedTicksStage1;
//        double didItSurpassWantedTicksStage2;
//        double didItSurpassWantedTicksStage3;
//        double didItSurpassWantedTicksStage4;
//        double didItSurpassWantedTicksStage5;
//        double didItSurpassWantedTicksStage6;
//        double didItSurpassWantedTicksStage7;
//        double didItSurpassWantedTicksStage8;
//        double didItSurpassWantedTicksStage9;
//        double didItSurpassWantedTicksStage10;
//        int numStagesUsed = 0;
//        int ticksLeftForStage10 = 0;
//        double stage10Seconds = 0;
//
//        // stage1 set-up
//        int Stage1Velocity = velocityAddUp * 1;
//        int Stage1TicksTraveled = (int) (Stage1Velocity * halfSecond);
//        totalTicksTraveled += Stage1TicksTraveled;
//        didItSurpassWantedTicksStage1 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
//                Stage1TicksTraveled, Stage1Velocity);
//        if (didItSurpassWantedTicksStage1 == halfSecond) {
//            reachedWantedTicks = false;
//        } else {
//            reachedWantedTicks = true;
//        }
//        StageInfoHolder.put("stage1Duration", didItSurpassWantedTicksStage1);
//        StageInfoHolder.put("stage1Velocity", (double) Stage1Velocity);
//        numStagesUsed += 1;
//
//        if (reachedWantedTicks == false) {
//            //stage2 set-up
//            int Stage2Velocity = velocityAddUp * 2;
//            int Stage2TicksTraveled = (int) (Stage2Velocity * halfSecond);
//            totalTicksTraveled += Stage2TicksTraveled;
//            didItSurpassWantedTicksStage2 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
//                    Stage2TicksTraveled, Stage2Velocity);
//            if (didItSurpassWantedTicksStage2 == halfSecond) {
//                reachedWantedTicks = false;
//            } else {
//                reachedWantedTicks = true;
//            }
//            StageInfoHolder.put("stage2Duration", didItSurpassWantedTicksStage2);
//            StageInfoHolder.put("stage2Velocity", (double) Stage2Velocity);
//            numStagesUsed += 1;
//
//            if (reachedWantedTicks == false) {
//                //stage3 set-up
//                int Stage3Velocity = velocityAddUp * 3;
//                int Stage3TicksTraveled = (int) (Stage3Velocity * halfSecond);
//                totalTicksTraveled += Stage3TicksTraveled;
//                didItSurpassWantedTicksStage3 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
//                        Stage3TicksTraveled, Stage3Velocity);
//                if (didItSurpassWantedTicksStage3 == halfSecond) {
//                    reachedWantedTicks = false;
//                } else {
//                    reachedWantedTicks = true;
//                }
//                StageInfoHolder.put("stage3Duration", didItSurpassWantedTicksStage3);
//                StageInfoHolder.put("stage3Velocity", (double) Stage3Velocity);
//                numStagesUsed += 1;
//
//                if (reachedWantedTicks == false) {
//                    //stage4 set-up
//                    int Stage4Velocity = velocityAddUp * 4;
//                    int Stage4TicksTraveled = (int) (Stage4Velocity * halfSecond);
//                    totalTicksTraveled += Stage4TicksTraveled;
//                    didItSurpassWantedTicksStage4 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
//                            Stage4TicksTraveled, Stage4Velocity);
//                    if (didItSurpassWantedTicksStage4 == halfSecond) {
//                        reachedWantedTicks = false;
//                    } else {
//                        reachedWantedTicks = true;
//                    }
//                    StageInfoHolder.put("stage4Duration", didItSurpassWantedTicksStage4);
//                    StageInfoHolder.put("stage4Velocity", (double) Stage4Velocity);
//                    numStagesUsed += 1;
//
////                    if (reachedWantedTicks == false) {
////                        //stage5 set-up
////                        ticksLeftForStage5 = wantedTicks - totalTicksTraveled;
////                        int Stage5Velocity = velocityAddUp * 5;
////                        stage5Seconds = this.moveForSeconds(ticksLeftForStage5, Stage5Velocity,
////                                true);
////                        StageInfoHolder.put("stage5Duration", stage5Seconds);
////                        StageInfoHolder.put("stage5Velocity", (double)Stage5Velocity);
////
////                    }
//                    if (reachedWantedTicks == false) {
//                        //stage4 set-up
//                        int Stage5Velocity = velocityAddUp * 5;
//                        int Stage5TicksTraveled = (int) (Stage5Velocity * halfSecond);
//                        totalTicksTraveled += Stage5TicksTraveled;
//                        didItSurpassWantedTicksStage5 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
//                                Stage5TicksTraveled, Stage5Velocity);
//                        if (didItSurpassWantedTicksStage5 == halfSecond) {
//                            reachedWantedTicks = false;
//                        } else {
//                            reachedWantedTicks = true;
//                        }
//                        StageInfoHolder.put("stage5Duration", didItSurpassWantedTicksStage5);
//                        StageInfoHolder.put("stage5Velocity", (double) Stage5Velocity);
//                        numStagesUsed += 1;
//
//                        if (reachedWantedTicks == false) {
//                            //stage4 set-up
//                            int Stage6Velocity = velocityAddUp * 6;
//                            int Stage6TicksTraveled = (int) (Stage6Velocity * halfSecond);
//                            totalTicksTraveled += Stage6TicksTraveled;
//                            didItSurpassWantedTicksStage6 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
//                                    Stage6TicksTraveled, Stage6Velocity);
//                            if (didItSurpassWantedTicksStage6 == halfSecond) {
//                                reachedWantedTicks = false;
//                            } else {
//                                reachedWantedTicks = true;
//                            }
//                            StageInfoHolder.put("stage6Duration", didItSurpassWantedTicksStage6);
//                            StageInfoHolder.put("stage6Velocity", (double) Stage6Velocity);
//                            numStagesUsed += 1;
//
//                            if (reachedWantedTicks == false) {
//                                //stage4 set-up
//                                int Stage7Velocity = velocityAddUp * 7;
//                                int Stage7TicksTraveled = (int) (Stage7Velocity * halfSecond);
//                                totalTicksTraveled += Stage7TicksTraveled;
//                                didItSurpassWantedTicksStage7 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
//                                        Stage7TicksTraveled, Stage7Velocity);
//                                if (didItSurpassWantedTicksStage7 == halfSecond) {
//                                    reachedWantedTicks = false;
//                                } else {
//                                    reachedWantedTicks = true;
//                                }
//                                StageInfoHolder.put("stage7Duration", didItSurpassWantedTicksStage7);
//                                StageInfoHolder.put("stage7Velocity", (double) Stage7Velocity);
//                                numStagesUsed += 1;
//
//                                if (reachedWantedTicks == false) {
//                                    //stage4 set-up
//                                    int Stage8Velocity = velocityAddUp * 5;
//                                    int Stage8TicksTraveled = (int) (Stage8Velocity * halfSecond);
//                                    totalTicksTraveled += Stage8TicksTraveled;
//                                    didItSurpassWantedTicksStage8 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
//                                            Stage8TicksTraveled, Stage8Velocity);
//                                    if (didItSurpassWantedTicksStage8 == halfSecond) {
//                                        reachedWantedTicks = false;
//                                    } else {
//                                        reachedWantedTicks = true;
//                                    }
//                                    StageInfoHolder.put("stage5Duration", didItSurpassWantedTicksStage8);
//                                    StageInfoHolder.put("stage5Velocity", (double) Stage8Velocity);
//                                    numStagesUsed += 1;
//
//                                    if (reachedWantedTicks == false) {
//                                        //stage4 set-up
//                                        int Stage9Velocity = velocityAddUp * 5;
//                                        int Stage9TicksTraveled = (int) (Stage9Velocity * halfSecond);
//                                        totalTicksTraveled += Stage9TicksTraveled;
//                                        didItSurpassWantedTicksStage9 = doesItSurpassTicks(totalTicksTraveled, wantedTicks, halfSecond,
//                                                Stage9TicksTraveled, Stage9Velocity);
//                                        if (didItSurpassWantedTicksStage9 == halfSecond) {
//                                            reachedWantedTicks = false;
//                                        } else {
//                                            reachedWantedTicks = true;
//                                        }
//                                        StageInfoHolder.put("stage5Duration", didItSurpassWantedTicksStage9);
//                                        StageInfoHolder.put("stage5Velocity", (double) Stage9Velocity);
//                                        numStagesUsed += 1;
//
//                                        if (reachedWantedTicks == false) {
//                                            //stage10 set-up
//                                            ticksLeftForStage10 = wantedTicks - totalTicksTraveled;
//                                            int Stage10Velocity = velocityAddUp * 5;
//                                            stage10Seconds = this.moveForSeconds(ticksLeftForStage10, Stage5Velocity,
//                                                    true);
//                                            StageInfoHolder.put("stage10Duration", stage10Seconds);
//                                            StageInfoHolder.put("stage10Velocity", (double) Stage10Velocity);
//                                            numStagesUsed += 1;
//
//
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//        StageInfoHolder.put("numStages", (double) numStagesUsed);
//
//        return StageInfoHolder;
//    }
//}
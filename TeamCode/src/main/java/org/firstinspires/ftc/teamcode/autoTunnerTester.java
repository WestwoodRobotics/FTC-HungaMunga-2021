package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="AutoTunnerTester", group="Iterative Opmode")
public class autoTunnerTester extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    double leftFrontVelocity = 0;
    double rightFrontVelocity = 0;
    double leftBackVelocity = 0;
    double rightBackVelocity = 0;
    int currentStage = 1;
    boolean initialPIDp = true;
    boolean initialPIDd = true;
    double initialPIDp1 = 15;
    double initialPIDp2 = 30;
    double initialPIDd1 = 15;
    double initialPIDd2 = 30;
    List<PIDclass> PIDpVals = new ArrayList<PIDclass>();
    List<PIDclass> PIDpValsStage2 = new ArrayList<PIDclass>();
    List<PIDclass> PIDdVals = new ArrayList<PIDclass>();
    int startTimeCounter = 0;
    double startTime = 0;
    double[] currentVelocities = new double[10];
    boolean alreadyChangedPIDval;
    AutoTunningMethods autoTunningMethods = new AutoTunningMethods();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //Setting Zero Power behaviour
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //setting PID coefficients
        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
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

    public boolean checkForRepeatsP(double wantedPVal) {
        for (int i = 0; i < PIDpVals.size(); i++) {
            PIDclass currentPIDp = PIDpVals.get(i);
            if (currentPIDp.getPIDVal() == wantedPVal) {
                return true;
            }
        }
        return false;
    }

    public boolean checkForRepeatsPStage2(double wantedPVal) {
        for (int i = 0; i < PIDpValsStage2.size(); i++) {
            PIDclass currentPIDp = PIDpValsStage2.get(i);
            if (currentPIDp.getPIDVal() == wantedPVal) {
                return true;
            }
        }
        return false;
    }

    public boolean getInput(double time) {
        if (time >= 500) {
            runtime.reset();
            return true;
        }
        else {
            return false;
        }
    }

    public PIDclass getWantedPIDp(double wantedPVal) {
        for (int i = 0; i < PIDpVals.size(); i++) {
            PIDclass currentPIDp = PIDpVals.get(i);
            if (currentPIDp.getPIDVal() == wantedPVal) {
                return currentPIDp;
            }
        }
        return null;
    }

    public int getWantedPIDpIndex(double wantedPVal) {
        for (int i = 0; i < PIDpVals.size(); i++) {
            PIDclass currentPIDp = PIDpVals.get(i);
            if (currentPIDp.getPIDVal() == wantedPVal) {
                return i;
            }
        }
        return 0;
    }

    public boolean goodPIDpFound() {
        for (int i = 0; i < PIDpVals.size(); i++) {
            PIDclass currentPIDp = PIDpVals.get(i);
            if (currentPIDp.getWins() == 2) {
                return true;
            }
        }
        return false;
    }

    public int findIndexOfBestPIDp() {
        for (int i = 0; i < PIDpVals.size(); i++) {
            PIDclass currentPIDp = PIDpVals.get(i);
            if (currentPIDp.getWins() == 2) {
                return i;
            }
        }
        return 0;
    }

    public PIDclass getBeterOf2(PIDclass pid1, PIDclass pid2) {
        if (pid1.getVelDiff() < pid2.getVelDiff()) {
            return pid1;
        }
        else {
            return pid2;
        }
    }

    public double getMultiplierFrom2(PIDclass startingPid, PIDclass secondPid) {
        double absoluteDif = Math.abs(startingPid.getPIDVal()-secondPid.getPIDVal());
        double multiplier = (startingPid.getPIDVal()-secondPid.getPIDVal())*(1.0/absoluteDif);
        return multiplier;
    }

    public double getBestPIDpOfStage2() {
        double bestPIDp = PIDpValsStage2.get(0).getVelDiff();
        for (int i = 0; i < PIDpValsStage2.size(); i++) {
            double currentStage2PIDp = PIDpValsStage2.get(i).getVelDiff();
            if (currentStage2PIDp < bestPIDp) {
                bestPIDp = currentStage2PIDp;
            }
        }
        return bestPIDp;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    boolean gotList1Done = false;
    boolean gotList2Done = false;
    boolean updatePIDp = false;
    boolean getNewDirection = false;
    double currentPIDp = 15;
    String wantedDirection = "MORE";
    PIDclass currentPIDp1;
    PIDclass currentPIDp2;
    int currentp1InfoIndex = 0;
    int currentp2InfoIndex = 0;
    double[] currentVelocities1 = new double[10];
    double[] currentVelocities2 = new double[10];
    double wantedVel = 3000;
    int currentWinner;
    boolean getSecondPIDp = false;
    int stage2Counter = 0;
    int PIDpstage2Counter = 0;
    double[] currentVelocitiesStage2 = new double[10];
    double currentPIDpS2 = 0;
    PIDclass currentPIDpClassS2;
    double bestPIDp = 0;
    boolean foundBestPIDp = false;
    boolean aboutToEnterStage2 = false;
    int timesRan = 0;

    @Override
    public void loop() {
        double currentTime = runtime.milliseconds();

        if (currentStage == 1) {
            if (!gotList1Done) {
                boolean checkForRepeats = checkForRepeatsP(currentPIDp);
                if (!checkForRepeats) {
                    // check if .5 of a seconds has passed to get another velocity value
                    boolean getNewInput = getInput(currentTime);
                    if (getNewInput && currentp1InfoIndex < 10) {
                        // adds it to list of velocities
                        currentVelocities1[currentp1InfoIndex] = rightBackDrive.getVelocity();
                        currentp1InfoIndex += 1;
                    }
                    if (currentp1InfoIndex == 10) {
                        // adds it to list of pid p values that have tested
                        currentPIDp1 = new PIDclass(currentPIDp, currentVelocities1, wantedVel);
                        PIDpVals.add(currentPIDp1);
                        telemetry.addData("addedValue", "i ran");
                        getSecondPIDp = true;
                        gotList1Done = true;
                        currentp1InfoIndex = 0;
                    }
                }
                else {
                    currentPIDp1 = getWantedPIDp(currentPIDp);
                    getSecondPIDp = true;
                    gotList1Done = true;
                    currentp1InfoIndex = 0;
                }
            }
            if (!gotList2Done) {
                boolean checkForRepeats = checkForRepeatsP(currentPIDp);
                if (!checkForRepeats) {
                    boolean getNewInput = getInput(currentTime);
                    if (getNewInput) {
                        currentVelocities2[currentp2InfoIndex] = rightBackDrive.getVelocity();
                        currentp2InfoIndex += 1;
                    }
                    if (currentp2InfoIndex == 10) {
                        currentPIDp2 = new PIDclass(currentPIDp, currentVelocities2, wantedVel);
                        PIDpVals.add(currentPIDp2);
                        gotList2Done = true;
                        currentp2InfoIndex = 0;
                    }
                }
                else {
                    currentPIDp2 = getWantedPIDp(currentPIDp);
                    gotList2Done = true;
                    currentp2InfoIndex = 0;
                }
            }
            else {
                getNewDirection = true;
            }

            if (getSecondPIDp) {
                if (wantedDirection.equals("More")) {
                    currentPIDp += 5;
                }
                else {
                    currentPIDp -= 5;
                }
                getSecondPIDp = false;
            }

            if (getNewDirection) {
                wantedDirection = autoTunningMethods.firstStageDirection(currentPIDp1, currentPIDp2);
                currentWinner = autoTunningMethods.getNewWinner();
                if (currentWinner <= 1) {
                    currentPIDp = currentPIDp1.getPIDVal();
                    int winnerPIDindex = getWantedPIDpIndex(currentPIDp);
                    PIDpVals.get(winnerPIDindex).updateWins();
                    gotList1Done = false;
                    gotList2Done = false;
                }
                else {
                    currentPIDp = currentPIDp2.getPIDVal();
                    int winnerPIDindex = getWantedPIDpIndex(currentPIDp);
                    PIDpVals.get(winnerPIDindex).updateWins();
                    gotList1Done = false;
                    gotList2Done = false;
                }
            }

            boolean moveToSecondsStage = goodPIDpFound();
            if (moveToSecondsStage) {
                currentStage = 2;
                aboutToEnterStage2 = true;
            }
            timesRan += 1;
        }
        else if (currentStage == 2){
            int bestPIDpIndex = findIndexOfBestPIDp();
            PIDclass bestPIDp = PIDpVals.get(bestPIDpIndex);
            PIDclass lessPIDp = getWantedPIDp(PIDpVals.get(bestPIDpIndex).getPIDVal()-5);
            PIDclass morePIDp = getWantedPIDp(PIDpVals.get(bestPIDpIndex).getPIDVal()+5);
            PIDclass secondsBestPIDp = getBeterOf2(lessPIDp, morePIDp);
            double directionFromSecond = getMultiplierFrom2(bestPIDp, secondsBestPIDp);
            double distanceFromPIDps = (Math.abs(bestPIDp.getPIDVal()-secondsBestPIDp.getVelDiff()));
            double distanceToChangeBy = distanceFromPIDps/4.0;
//            boolean checkForBestRepetitions = checkForRepeatsPStage2(bestPIDp.getPIDVal());
//            currentPIDpS2 = bestPIDp.getPIDVal();
//            if (!checkForBestRepetitions) {
//                PIDpValsStage2.add(bestPIDp);
//            }

            currentPIDp = (secondsBestPIDp.getPIDVal())+(stage2Counter*distanceToChangeBy);
            if (stage2Counter < 4) {
                if (PIDpstage2Counter < 10) {
                    boolean checkForRepetitions = checkForRepeatsPStage2(currentPIDpS2);
                    if (!checkForRepetitions) {
                        boolean getNewInput = getInput(currentTime);
                        if (getNewInput) {
                            currentVelocitiesStage2[PIDpstage2Counter] = rightBackDrive.getVelocity();
                            PIDpstage2Counter += 1;
                        }
                    }
                    else {
                        currentPIDpClassS2 = new PIDclass(currentPIDpS2, currentVelocitiesStage2, wantedVel);
                        PIDpstage2Counter = 0;
                    }
                }
                else {
                    currentPIDpClassS2 = new PIDclass(currentPIDpS2, currentVelocitiesStage2, wantedVel);
                    PIDpValsStage2.add(currentPIDpClassS2);
                    PIDpstage2Counter = 0;
                    stage2Counter += 1;
                }
            }
            else {
                currentStage = 3;
            }
        }
        else if (currentStage == 3) {
            bestPIDp = getBestPIDpOfStage2();
            foundBestPIDp = true;
        }

        if (!foundBestPIDp) {
            rightBackDrive.setVelocityPIDFCoefficients(currentPIDp,1,1,1);
        }
        else {
            rightBackDrive.setVelocityPIDFCoefficients(bestPIDp,1,1,1);
        }

        rightBackDrive.setVelocity(3000);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f)", leftFrontDrive.getVelocity());
        telemetry.addData("Motors", "right front (%.2f)", rightFrontDrive.getVelocity());
        telemetry.addData("Motors", "left back (%.2f)", leftBackDrive.getVelocity());
        telemetry.addData("Motors", "right back (%.2f)", rightBackDrive.getVelocity());
        telemetry.addData("final PID", "finalPIDp: " + bestPIDp);
        telemetry.addData("currentStage", currentStage);
        telemetry.addData("aboutToEnterStage2", aboutToEnterStage2);
        telemetry.addData("pidListSize", PIDpVals.size());
        for (int i = 0; i < PIDpVals.size(); i++) {
            telemetry.addData("pidpVal: ", PIDpVals.get(i));
        }
        telemetry.addData("timesRan", timesRan);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

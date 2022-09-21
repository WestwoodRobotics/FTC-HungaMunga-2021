//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.checkerframework.checker.units.qual.A;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//@TeleOp(name="V3AutoTunnerTester", group="Iterative Opmode")
//public class V3AutoTunnerTester extends OpMode
//{
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotorEx leftFrontDrive = null;
//    private DcMotorEx rightFrontDrive = null;
//    private DcMotorEx leftBackDrive = null;
//    private DcMotorEx rightBackDrive = null;
//
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//        telemetry.addData("Status", "Initialized");
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
//        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
//        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
//        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");
//
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
//
//        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
//
//        //Setting Zero Power behaviour
//        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        //setting PID coefficients
//        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        rightFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    @Override
//    public void init_loop() {
//    }
//
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//        runtime.reset();
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//     */
//
//    public int findBestIndex(ArrayList<UpdatesPID> PIDps) {
//        ArrayList<UpdatesPID> copyList = new ArrayList<>(PIDps);
//        double bestDiff = copyList.get(0).getTotalAvgDiff();
//        int bestIndex = 0;
//        for (int i = 0; i < copyList.size(); i++) {
//            double currentVelDiff = copyList.get(i).getTotalAvgDiff();
//            if (currentVelDiff < bestDiff) {
//                bestDiff = currentVelDiff;
//                bestIndex = i;
//            }
//        }
//        return bestIndex;
//    }
//
//    public int findSecondBestIndex(ArrayList<UpdatesPID> PIDps) {
//        ArrayList<UpdatesPID> copyList = new ArrayList<>(PIDps);
//        double bestDiff = copyList.get(0).getTotalAvgDiff();
//        int bestIndex = 0;
//        for (int i = 0; i < copyList.size(); i++) {
//            double currentVelDiff = copyList.get(i).getTotalAvgDiff();
//            if (currentVelDiff < bestDiff) {
//                bestDiff = currentVelDiff;
//                bestIndex = i;
//            }
//        }
//
//        copyList.remove(bestIndex);
//        double bestDiff2 = copyList.get(0).getTotalAvgDiff();
//        int secondBestIndex = 0;
//        for (int k = 0; k < copyList.size(); k++) {
//            double currentVelDiff2 = copyList.get(k).getTotalAvgDiff();
//            if (currentVelDiff2 < bestDiff2) {
//                bestDiff2 = currentVelDiff2;
//                secondBestIndex = k;
//            }
//        }
//
//        return secondBestIndex;
//    }
//
//    public void setHighLow(UpdatesPID bestPIDp, UpdatesPID secondBestPIDp) {
//        UpdatesPID best = new UpdatesPID(bestPIDp.getPidVal(),
//                bestPIDp.getVelocities(), bestPIDp.getWantedVels(), bestPIDp.getWantedCoveredVels());
//        UpdatesPID second = new UpdatesPID(secondBestPIDp.getPidVal(),
//                secondBestPIDp.getVelocities(), secondBestPIDp.getWantedVels(), secondBestPIDp.getWantedCoveredVels());
//        if (best.getPidVal() < second.getPidVal()) {
//            lower = best;
//            higher = second;
//        }
//        else {
//            lower = second;
//            higher = best;
//        }
//    }
//
//    // velocities being tested
//    // 2000/50 = 40
//    // 4 check per velocity
//    private double currentPidP = 5;
//    private int counter = 0;
//    private ArrayList<PIDclass> allPidPs = new ArrayList<PIDclass>();
//    private ArrayList<PIDclass> bestPIDps = new ArrayList<PIDclass>();
//    private ArrayList<UpdatesPID> allPIDupdated = new ArrayList<UpdatesPID>();
//    private ArrayList<UpdatesPID> bestPIDupdated = new ArrayList<UpdatesPID>();
//    private double[] wantedVels = new double[50];
//    private double[] currentVelocities = new double[160];
//    private int addToVelCounter = 0;
//    private int wantedVel = 40;
//    private int totalCounters = 160;
//    private int newVelCounter = 4;
//    private int velCounter = 0;
//    private boolean activateInbetweenTime = false;
//    private int inbetweenTime = 0;
//    private boolean activateInbetweenTime2 = false;
//    private int inbetweenTime2 = 0;
//    private int totalInbetweenTime = 50;
//
//    private UpdatesPID lower;
//    private UpdatesPID higher;
//    private boolean addSecondStageInitialPIDps = true;
//    private int stage2Counter = 0;
//    private int stage2InbTime = 0;
//    private boolean firstS2Val = true;
//    private boolean doStage1 = true;
//    private boolean doStage2 = false;
//    private boolean doStage3 = false;
//    private double bestPIDp = 0;
//
//
//    @Override
//    public void loop() {
//        telemetry.addData("test: ", currentVelocities.length);
//        telemetry.addData("dostage2: ", doStage2);
//        telemetry.addData("dostage1: ", doStage1);
//        telemetry.addData("dostage3: ", doStage3);
//        telemetry.addData("currentPIDp: ", currentPidP);
//        for (int i = 0; i < bestPIDps.size(); i++) {
//            telemetry.addData("element" + i + ": ", bestPIDps.get(i).getPIDVal());
//        }
//
//
//        //telemetry.addData("doSTage1: ", doStage1);
//        if (doStage1) {
//            if (activateInbetweenTime) {
//                if (inbetweenTime < totalInbetweenTime) {
//                    inbetweenTime += 1;
//                }
//                else {
//                    activateInbetweenTime = false;
//                    inbetweenTime = 0;
//                }
//            }
//            else {
//                if (velCounter < newVelCounter) {
//                    velCounter += 1;
//                }
//                else {
//                    velCounter = 0;
//                    wantedVels[addToVelCounter] = wantedVel;
//                    addToVelCounter += 1;
//                    wantedVel += 40;
//                    if (wantedVel > 2000) {
//                        wantedVel = 0;
//                    }
//                }
//                if (counter < totalCounters) {
//                    //telemetry.addData("counter: ", counter);
//                    currentVelocities[counter] = rightBackDrive.getVelocity();
//                    counter += 1;
//                }
//                else {
//                    UpdatesPID newPIDp = new UpdatesPID(currentPidP, currentVelocities, wantedVels, newVelCounter);
//                    allPIDupdated.add(newPIDp);
//                    currentPidP += 5;
//                    counter = 0;
//                    activateInbetweenTime = true;
//                    if (allPidPs.size() == 8) {
//                        doStage1 = false;
//                        doStage2 = true;
//                    }
//                }
//            }
//        }
//        else {
//            if (doStage2) {
//                UpdatesPID bestPidp = allPIDupdated.get(this.findBestIndex(allPIDupdated));
//                UpdatesPID secondBestPidp = allPIDupdated.get(this.findSecondBestIndex(allPIDupdated));
//                if (addSecondStageInitialPIDps) {
//                    bestPIDupdated.add(bestPidp);
//                    bestPIDupdated.add(secondBestPidp);
//                    addSecondStageInitialPIDps = false;
//                }
//
//                this.setHighLow(bestPidp, secondBestPidp);
//                double difference = higher.getPidVal() - lower.getPidVal();
//                double addValue = difference/3.0;
//
//                if (firstS2Val) {
//                    currentPidP = lower.getPidVal() + addValue;
//                    firstS2Val = false;
//                }
//
//                if (activateInbetweenTime2) {
//                    if (inbetweenTime2 < totalInbetweenTime) {
//                        inbetweenTime2 += 1;
//                    }
//                    else {
//                        activateInbetweenTime2 = false;
//                        inbetweenTime2 = 0;
//                    }
//                }
//                else {
//                    if (velCounter < newVelCounter) {
//                        velCounter += 1;
//                    }
//                    else {
//                        velCounter = 0;
//                        wantedVels[addToVelCounter] = wantedVel;
//                        addToVelCounter += 1;
//                        wantedVel += 40;
//                        if (wantedVel > 2000) {
//                            wantedVel = 0;
//                        }
//                    }
//                    if (stage2Counter < totalCounters) {
//                        currentVelocities[stage2Counter] = rightBackDrive.getVelocity();
//                        stage2Counter += 1;
//                    }
//                    else {
//                        UpdatesPID newPIDp = new UpdatesPID(currentPidP, currentVelocities, wantedVels, newVelCounter);
//                        allPIDupdated.add(newPIDp);
//                        currentPidP += addValue;
//                        stage2Counter = 0;
//                        activateInbetweenTime = true;
//                        if (bestPIDps.size() > 4) {
//                            doStage2 = false;
//                            doStage3 = true;
//                        }
//                    }
//                }
//            }
//
//        }
//        if (doStage3 == true) {
//            int indexOfBest = findBestIndex(bestPIDps);
//            currentPidP = bestPIDps.get(indexOfBest).getPIDVal();
//            bestPIDp = currentPidP;
//        }
//
//        rightBackDrive.setVelocityPIDFCoefficients(40, 5, 20, 25);
//        rightBackDrive.setVelocity(wantedVel);
//
//        telemetry.addData("allPidp size", allPidPs.size());
//
////        telemetry.addData("pid list", allPidPs.size());
////        telemetry.addData("best pid list", bestPIDps.size());
////        telemetry.addData("current P: ", currentPidP);
//        if (doStage3 == true) {
//            telemetry.addData("best PIDp: ", bestPIDp);
//        }
//        telemetry.addData("velocity: ", rightBackDrive.getVelocity());
//
//    }
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//    }
//
//}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDclass {
    private double pidVal;
    private double[] velocities;
    private double averageVelocity;
    private double velDiff;
    private int pidCounter;
    private int wins;
    private double wantedVel;

    public PIDclass() {

    }

    public PIDclass(double pidVal, double[] velocities, double wantedVel) {
        this.wantedVel = wantedVel;
        this.pidVal = pidVal;
        this.velocities = velocities;
        this.averageVelocity = calculateAverageVelocity(velocities);
        velDiff = Math.abs(wantedVel - averageVelocity);
        pidCounter = 0;
    }

    // returns the average velocity over a certain amount of velocities that are gathered
    public double calculateAverageVelocity(double[] velocities) {
        double totalvelocity = 0;
        double average = 0;
        for (int i = 0; i < velocities.length; i++) {
            totalvelocity += velocities[i];
        }
        average = totalvelocity/velocities.length;
        return average;
    }

    public double getWantedVel() {
        return wantedVel;
    }

    public double[] getVelocities() {
        return velocities;
    }

    public double getPIDVal() {
        return pidVal;
    }

    public double getVelDiff() {
        return velDiff;
    }

    public int getPidCounter() {
        return pidCounter;
    }

    public int getWins(){
        return wins;
    }

    public void setPidCounter(int newPID) {
        pidCounter = newPID;
    }

    public void updateWins() {
        wins += 1;
    }
}

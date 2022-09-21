package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class UpdatesPID {
    private double pidVal;
    private double[] velocities;
    private double[] wantedVels;
    private int wantedCoveredVels;
    private double totalAvgDiff;

    public UpdatesPID() {

    }

    public UpdatesPID(double pidVal, double[] velocities, double[] wantedVels,
                      int wantedCoveredVels) {

        this.pidVal = pidVal;
        this.velocities = velocities;
        this.wantedVels = wantedVels;
        this.wantedCoveredVels = wantedCoveredVels;
        this.totalAvgDiff = calculateAverageVelDiff();
    }

    public double calculateAverageVelDiff() {
        int totalAvgDiff = 0;
        int totalDiffs = velocities.length/wantedCoveredVels;
        double finalAvgDiff = 0;
        for (int avgDiffs = 0; avgDiffs < totalDiffs; avgDiffs++) {
            double curTotalVels = 0;
            for (int i = 0; i < wantedCoveredVels; i++) {
                curTotalVels += Math.abs(velocities[i]-wantedVels[avgDiffs]);
            }
            totalAvgDiff += (curTotalVels/wantedCoveredVels);
        }
        finalAvgDiff = (totalAvgDiff/wantedCoveredVels);
        return finalAvgDiff;
    }

    public double getPidVal() {
        return pidVal;
    }

    public double[] getVelocities() {
        return velocities;
    }

    public double[] getWantedVels() {
        return wantedVels;
    }

    public int getWantedCoveredVels() {
        return wantedCoveredVels;
    }

    public double getTotalAvgDiff() {
        return totalAvgDiff;
    }
}


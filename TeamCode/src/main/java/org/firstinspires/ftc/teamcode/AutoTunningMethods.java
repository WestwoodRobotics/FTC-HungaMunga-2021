package org.firstinspires.ftc.teamcode;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

public class AutoTunningMethods {
    private int numDifference;
    private String MoreorLess;
    private double velocity;
    private int updatePID1;
    private int updatePID2;
    private int currentWinner;


    public AutoTunningMethods() {
    }


    public AutoTunningMethods(int initialChange, double velocity) {
        this.velocity = velocity;
    }

    public String firstStageDirection(PIDclass pid1, PIDclass pid2) {
        if (pid1.getVelDiff() < pid2.getVelDiff()) {
            if (pid1.getPIDVal() < pid1.getPIDVal()) {
                MoreorLess = "LESS";
            }
            else {
                MoreorLess = "More";
            }
            updatePID1 = pid1.getPidCounter() + 1;
            updatePID2 = 0;
            currentWinner = 1;
            return MoreorLess;
        }
        else {
            if (pid2.getPIDVal() < pid1.getPIDVal()) {
                MoreorLess = "LESS";
            }
            else {
                MoreorLess = "MORE";
            }
            updatePID2 = pid2.getPidCounter() + 1;
            updatePID1 = 0;
            currentWinner = 2;
            return MoreorLess;
        }
    }

    public int getNewWinner() {
        return currentWinner;
    }

    public int getUpdatedPID1Counter() {
        return updatePID1;
    }

    public int getUpdatedPID2Counter() {
        return updatePID2;
    }

    public int addedValFirstStage(String wantedDirection) {
        if (wantedDirection.equals("MORE")) {
            return 5;
        }
        else {
            return -5;
        }
    }

    public double SecondStageNewPID(List<PIDclass> rangeVals) {
        List<PIDclass> newRangeUnordered = getBest2Vals(rangeVals);
        List<PIDclass> newRangeOrdered = putInorder(newRangeUnordered);
        double pidIndex0 = newRangeOrdered.get(0).getPIDVal();
        double pidIndex1 = newRangeOrdered.get(1).getPIDVal();
        double newPID = pidIndex0+((pidIndex1 - pidIndex0)/2);
        return newPID;
    }

    public List<PIDclass> getBest2Vals(List<PIDclass> rangeVals) {
        List<PIDclass> rangeOf2 = new ArrayList<PIDclass>();
        List<PIDclass> editableRange = new ArrayList<PIDclass>();
        for (int i = 0; i < rangeVals.size(); i++) {
            editableRange.add(rangeVals.get(i));
        }
        int number1best = getBestval(editableRange);
        rangeOf2.set(0, editableRange.get(number1best));
        editableRange.remove(number1best);
        int number2best = getBestval(editableRange);
        rangeOf2.set(1, editableRange.get(number2best));
        editableRange.remove(number2best);
        return rangeOf2;
    }

    public int getBestval(List<PIDclass> rangeVals) {
        int bestValIndex = 0;
        for (int i = 0; i < rangeVals.size(); i++) {
            if (rangeVals.get(i).getVelDiff() < bestValIndex) {
                bestValIndex = i;
            }
        }
        return bestValIndex;
    }

    public List<PIDclass> putInorder(List<PIDclass> unorderedList) {
        List<PIDclass> orderedList = new ArrayList<PIDclass>();
        double pidIndex0 = unorderedList.get(0).getPIDVal();
        double pidIndex1 = unorderedList.get(1).getPIDVal();

        if (pidIndex0 < pidIndex1) {
            return unorderedList;
        }
        else {
            orderedList.set(0, unorderedList.get(1));
            orderedList.set(1, unorderedList.get(0));
            return orderedList;
        }
    }

}

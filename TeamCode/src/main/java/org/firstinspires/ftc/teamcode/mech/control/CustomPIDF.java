package org.firstinspires.ftc.teamcode.mech.control;

import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;
import java.lang.Math;
/**
 * Units:
 *  - target/measured are in ticks/sec
 *  - output is motor power [-1, 1]
 */
public class CustomPIDF {
    public double kP, kI, kD, kF;

    // safety
    public double iMax = 0.4;          // max magnitude of integral contribution
    public double outputMin = -1.0;
    public double outputMax =  1.0;

    public ArrayList<Double> errorlist;
    public ArrayList<Double> timelist;
    private ArrayList<Double> stdlist;
    
    //This is 0 when we are oslating (or close to oslating)
    public double osalationratio =100;
    
    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean hasLast = false;

    public CustomPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        errorlist = new ArrayList<>();
        timelist = new ArrayList<>();
        stdlist = new ArrayList<>();
    
        osalationratio =100;
        
    }
    //Calculates the spread of the data set, helps show the oslatation per new value, n stuff
    public double StandardDeviationError(){
        double sum = 0;
        double mean = 0;
        double STD = 0;
        for(int i = 0; i<errorlist.size(); i++){
            sum+=errorlist.get(i);
        }
        mean = sum/errorlist.size();
        for (int i = 0; i<errorlist.size(); i++) {
            STD += Math.pow(errorlist.get(i) - mean, 2);
        }
        return Math.sqrt(STD / (errorlist.size() - 1));
    }
    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        hasLast = false;
    }

//Actualy ts is ziegler nichloas testing for a single Kp Value, we only use kp and wait till we have osalation, which would man that the STD is fairly constnat aka 0

public double ZiegerZichloas(double targetTicksPerSec, double measuredTicksPerSec, double dtSec) {
        if (dtSec <= 1e-6) dtSec = 1e-3;

        double error = targetTicksPerSec - measuredTicksPerSec;

        errorlist.add(error);
        stdlist.add(StandardDeviationError());
        timelist.add(timelist.get(-1)+dtSec);
        if(stdlist.size()>3){
        osalationratio = Math.log((stdlist.get(stdlist.size()-1))/(stdlist.get(stdlist.size()-2)));
        }

        lastError = error;
        hasLast = true;

        double out = (kP * error);
        return Range.clip(out, outputMin, outputMax);
    }
    
    public double update(double targetTicksPerSec, double measuredTicksPerSec, double dtSec) {
        if (dtSec <= 1e-6) dtSec = 1e-3;

        double error = targetTicksPerSec - measuredTicksPerSec;
        
        
        // Integral with clamp
        integral += error * dtSec;
        double iTerm = kI * integral;
        iTerm = Range.clip(iTerm, -iMax, iMax);

        // Derivative on error
        double dTerm = 0.0;
        if (hasLast) {
            double derivative = (error - lastError) / dtSec;
            dTerm = kD * derivative;
        }
        lastError = error;
        hasLast = true;

        // Feedforward: power needed for target velocity (roughly)
        double fTerm = kF * targetTicksPerSec;

        double out = (kP * error) + iTerm + dTerm + fTerm;
        return Range.clip(out, outputMin, outputMax);
    }
}

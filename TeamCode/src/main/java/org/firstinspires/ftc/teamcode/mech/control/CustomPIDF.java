package org.firstinspires.ftc.teamcode.mech.control;

import com.qualcomm.robotcore.util.Range;

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

    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean hasLast = false;

    public CustomPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        hasLast = false;
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

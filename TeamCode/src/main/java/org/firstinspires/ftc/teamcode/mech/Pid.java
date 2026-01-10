package org.firstinspires.ftc.teamcode.mech;

class Pid{
    private double kP, kI, kD;
    private long setpoint = 2200 ;
    private double integralSum = 0;
    private double lastError = 0;
    private double outputMin, outputMax;
    private double period; // Time between updates

    public void PIDController(double kP, double kI, double kD, double period) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.period = period;
        // Set default output limits if needed
        this.outputMin = 0.0;
        this.outputMax = 1.0;
    }

    public void setSetpoint(long setpoint) {
        this.setpoint = setpoint;
    }

    public void setOutputRange(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    public double calculate(double processValue) {
        // Calculate error
        double error = setpoint - processValue;

        // Proportional term
        double proportional = kP * error;

        // Integral term
        integralSum += (error * period);
        // Optional: Clamp the integral sum to prevent windup
        integralSum = Math.max(outputMin, Math.min(integralSum, outputMax));

        // Derivative term
        double derivative = (error - lastError) / period;
        lastError = error;

        // Calculate total output
        double output = proportional + (kI * integralSum) + (kD * derivative);

        // Clamp output to defined range
        output = Math.max(outputMin, Math.min(output, outputMax));

        return output;
    }
}
package org.firstinspires.ftc.teamcode.mech.mainIntakeOuttake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Config
public class launcher {

    /** Desired launcher speed as a fraction of the motor's configured max TPS. */
    public static double SPEED_FRACTION = 0.12;

    /** PIDF on velocity error (units: power per (ticks/sec)). */
    public static double kP = 0.0006;
    public static double kI = 0.00025;
    public static double kD = 0.000001              ;
    /** Feedforward on target velocity (power per (ticks/sec)). Start with 0, then tune. */
    public static double kF = 0.0;
    /** Accel feedforward (power per (ticks/sec^2)). Start with 0, then tune. */
    public static double kA = 0.0;

    /** Only integrate when |error| is smaller than this (ticks/sec). */
    public static double I_ZONE_TPS = 5000;
    /** Integral clamp (power units / kI). */
    public static double I_MAX = 0.5;

    /** Consider "at speed" when within this tolerance (ticks/sec). */
    public static double AT_SPEED_TOL_TPS = 5;

    /** Max absolute power the controller is allowed to command. */
    public static double MAX_POWER = 1.0;

    /**
     * Rev-up boost: when target speed increases, temporarily multiply the target for BOOST_MS.
     * Helps the wheel accelerate faster.
     */
    public static double BOOST_MULT = 1.15;
    public static double BOOST_MS = 100;

    private final DcMotorEx motor;
    private final ElapsedTime clock = new ElapsedTime();

    private final double maxTicksPerSec;

    private double targetTicksPerSec = 0.0;
    private double boostUntilMs = 0.0;

    private double lastTimeSec = 0.0;
    private double lastError = 0.0;
    private double integral = 0.0;
    private double lastTargetForAccel = 0.0;
    private double lastPowerCmd = 0.0;

    public launcher(LinearOpMode opMode) {
        this(opMode, "launcher");
    }

    public launcher(LinearOpMode opMode, String hardwareName) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, hardwareName);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Flywheels usually prefer FLOAT so they coast instead of braking hard.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // We'll run our own PID (still reading encoders), so don't use built-in velocity mode.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Estimate max ticks/sec from motor configuration (used for SPEED_FRACTION).
        MotorConfigurationType type = motor.getMotorType();
        double maxRpm = type.getMaxRPM() * type.getAchieveableMaxRPMFraction();
        maxTicksPerSec = (maxRpm / 60.0) * type.getTicksPerRev();

        clock.reset();
        lastTimeSec = clock.seconds();
    }

    /** Rev to the dashboard-configured SPEED_FRACTION. */
    public void revDefault() {
        revFraction(SPEED_FRACTION);
    }

    // Backwards-compatible helpers (for older code that called on/off)
    public void on() {
        revDefault();
    }

    public void off() {
        stop();
    }

    /** Rev to a fraction (0..1) of the motor's configured max ticks/sec. */
    public void revFraction(double fraction) {
        fraction = Range.clip(fraction, 0.0, 1.0);
        revTicksPerSec(fraction * maxTicksPerSec);
    }

    /** Rev to an target velocity in encoder ticks/sec */
    public void revTicksPerSec(double ticksPerSec) {
        ticksPerSec = Math.max(0.0, ticksPerSec);
        // Apply a short boost if we're increasing speed.
        if (ticksPerSec > targetTicksPerSec + 1e-6) {
            boostUntilMs = clock.milliseconds() + BOOST_MS;
        }
        targetTicksPerSec = ticksPerSec;
    }

    /** Stop the launcher and clear controller state */
    public void stop() {
        targetTicksPerSec = 0.0;
        boostUntilMs = 0.0;
        integral = 0.0;
        lastError = 0.0;
        lastTargetForAccel = 0.0;
        lastPowerCmd = 0.0;
        motor.setPower(0.0);
    }

    /** update PID output. */
    public void update() {
        if (targetTicksPerSec <= 0.0) {
            // Keep it off when not in use.
            motor.setPower(0.0);
            lastPowerCmd = 0.0;
            integral = 0.0;
            lastError = 0.0;
            lastTargetForAccel = 0.0;
            return;
        }

        double nowSec = clock.seconds();
        double dt = nowSec - lastTimeSec;
        if (dt <= 1e-4) dt = 0.02;
        lastTimeSec = nowSec;

        double current = motor.getVelocity(); // ticks/sec

        double tgt = targetTicksPerSec;
        if (clock.milliseconds() <= boostUntilMs) {
            tgt = targetTicksPerSec * BOOST_MULT;
        }

        double error = tgt - current;
        double deriv = (error - lastError) / dt;
        lastError = error;

        if (Math.abs(error) < I_ZONE_TPS) {
            integral += error * dt;
            integral = Range.clip(integral, -I_MAX, I_MAX);
        } else {
            // Prevent windup when far away.
            integral = 0.0;
        }

        double accel = (tgt - lastTargetForAccel) / dt;
        lastTargetForAccel = tgt;

        double power = (kP * error) + (kI * integral) + (kD * deriv) + (kF * tgt) + (kA * accel);
        power = Range.clip(power, -MAX_POWER, MAX_POWER);

        lastPowerCmd = power;
        motor.setPower(power);
    }
    public boolean atSpeed() {
        if (targetTicksPerSec <= 0.0) return false;
        return Math.abs(targetTicksPerSec - motor.getVelocity()) <= AT_SPEED_TOL_TPS;
    }

    public double getVelocityTicksPerSec() {
        return motor.getVelocity();
    }

    public double getTargetTicksPerSec() {
        return targetTicksPerSec;
    }

    public double getLastPowerCmd() {
        return lastPowerCmd;
    }

    public double getMaxTicksPerSec() {
        return maxTicksPerSec;
    }
}

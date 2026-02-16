package org.firstinspires.ftc.teamcode.mech.control;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretController {

    // YAW SERVO GEAR RATIOS: 45:132
    // PITCH SERVO GEAR RATIOS: 26:200
    // Hardware
    private final CRServo yawServo;
    private final Servo pitchServo;

    // Optional analog yaw encoder (e.g., Axon servo encoder)
    private final AnalogInput yawEncoder;
    private final boolean hasYawEncoder;

    // If turret direction is flipped, set true
    public boolean yawInverted = false;

    // Safety clamp for max yaw power
    public double yawMaxPower = 1.0;
    private double filteredYawErrDeg = 0.0;

    // Approximate turret angular speed (deg/sec) when yawServo is commanded at full power (1.0).
    public double yawDegPerSecAtFullPower = 178.0;


    // encoder configuration
    // 0-3.3V over one mechanical revolution.
    public double yawEncoderMaxVoltage = 3.3;
    // Turret degrees represented by one full encoder revolution.
    public double yawEncoderDegPerRev = 122.7272;
    // Additive offset applied after unwrapping, in degrees.
    public double yawEncoderOffsetDeg = 0.0;
    public boolean yawEncoderInverted = false;

    // Unwrapped encoder state (continuous degrees, without offset)
    private double yawEncLastRawDeg = 0.0;
    private double yawEncContinuousDeg = 0.0;
    private boolean yawEncHasLast = false;

    // Position PIDF for yaw hold/aim (deg -> power)
    private final CustomPIDF yawPidf;

    private double targetAngleDeg = 0.0;
    private double visionDistanceIn = 0.0;   // still used for pitch lookup
    private boolean targetValid = false;
    private long lastTargetTime = 0;
    public long targetTimeoutMs = 500;

    // Pitch table (distance in inches to servo position)
    // Must be same length and strictly increasing distances.
    public double[] pitchDistIn = { 18, 30, 42, 54 };
    public double[] pitchPos    = {0.78,0.70,0.64,0.60};

    // Hard clamps for safety
    public double pitchMinPos = 0.4;
    public double pitchMaxPos = 0.8;

    // Slew-rate to prevent pitch oscillations
    public double pitchSlewPerSec = 1.5;

    // Aim tolerance + settle time
    public double aimTolYawDeg = 1.5;
    public double aimTolPitchPos = 0.02;
    public long settleMs = 120;

    // Target point relative to robot (optional; used for pitch if vis ion distance is not valid)
    private double targetXIn = 0;   // forward
    private double targetYIn = 0;   // left
    private double targetZIn = 0;   // up (optional)

    // Internal pitch state
    private double pitchCmd;
    private double pitchDesired;

    // Internal yaw state (deg)
    private double yawEstimateDeg = 0.0;
    private double yawTargetDeg = 0.0;

    private double out;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    private double tx;

    public TurretController(CRServo yawServo, Servo pitchServo) {
        this(yawServo, pitchServo, null);
    }

    public TurretController(CRServo yawServo, Servo pitchServo, AnalogInput yawEncoder) {
        this.yawServo = yawServo;
        this.pitchServo = pitchServo;

        this.yawEncoder = yawEncoder;
        this.hasYawEncoder = (yawEncoder != null);

        // Position PID defaults (YOU WILL NEED TO TUNE)
        this.yawPidf = new CustomPIDF(0.0005, 0.0, 0.0, 0.0);
        this.yawPidf.iMax = 0.0;

        pitchCmd = pitchServo.getPosition();
        pitchDesired = pitchCmd;

        loopTimer.reset();
        settleTimer.reset();
    }

    /** Zero the internal yaw estimate/target */
    public void resetYawEstimate(double yawDeg) {
        // If we have an encoder, set the offset so the encoder reading equals yawDeg.
        if (hasYawEncoder) {
            yawEncHasLast = false;
            double encNow = readYawEncoderDeg();
            yawEncoderOffsetDeg += (yawDeg - encNow);
        }
        yawEstimateDeg = yawDeg;
        yawTargetDeg = yawDeg;
        yawPidf.reset();
        settleTimer.reset();
    }

    /** Convenience: set both estimate and target to 0 deg. */
    public void resetYawEstimate() {
        resetYawEstimate(0.0);
    }

    public double getYawEstimateDeg() { return yawEstimateDeg; }
    public double getYawTargetDeg() { return yawTargetDeg; }

    public void setTargetRobotRelative(double xIn, double yIn, double zIn) {
        this.targetXIn = xIn;
        this.targetYIn = yIn;
        this.targetZIn = zIn;
        settleTimer.reset();
    }

    /**
     * Update targeting for yaw + pitch.
     *
     * targetAngleDeg: desired turret yaw angle in ROBOT frame (deg). 0 = forward, + = left (CCW).
     * distanceIn: distance to target (inches) for pitch interpolation.
     * isValid: whether the target measurement is valid/fresh.
     */
    public void updateTargetMeasurement(double targetAngleDeg, double distanceIn, boolean isValid) {
        this.targetAngleDeg = targetAngleDeg;
        this.visionDistanceIn = distanceIn;
        this.targetValid = isValid;
        if (isValid) {
            this.lastTargetTime = System.currentTimeMillis();
        }
    }

    public void updateVisionMeasurement(double yawErrorDeg, double distanceIn, boolean isValid) {
        updateTargetMeasurement(yawErrorDeg, distanceIn, isValid);
    }

    
    /** Read the analog yaw encoder and return a continuous turret angle in degrees. */
    private double readYawEncoderDeg() {
        double v = yawEncoder.getVoltage();
        double raw = (v / Math.max(1e-6, yawEncoderMaxVoltage)) * yawEncoderDegPerRev;

        // Wrap
        raw = raw % yawEncoderDegPerRev;
        if (raw < 0) raw += yawEncoderDegPerRev;

        if (yawEncoderInverted) {
            raw = yawEncoderDegPerRev - raw;
            if (raw >= yawEncoderDegPerRev) raw -= yawEncoderDegPerRev;
        }

        // Unwrap to continuous angle
        if (!yawEncHasLast) {
            yawEncHasLast = true;
            yawEncLastRawDeg = raw;
            yawEncContinuousDeg = raw;
        } else {
            double delta = raw - yawEncLastRawDeg;
            double half = yawEncoderDegPerRev / 2.0;
            if (delta > half) delta -= yawEncoderDegPerRev;
            if (delta < -half) delta += yawEncoderDegPerRev;
            yawEncContinuousDeg += delta;
            yawEncLastRawDeg = raw;
        }

        return yawEncContinuousDeg + yawEncoderOffsetDeg;
    }

public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 1e-6) dt = 0.02;

        // Update yaw estimate from encoder when available otherwise integrate
        if (hasYawEncoder) {
            yawEstimateDeg = readYawEncoderDeg();
        }

        double dist = Math.hypot(targetXIn, targetYIn);
        if (targetValid) dist = visionDistanceIn;

        if (Math.abs(targetZIn) > 0.5) {
            double elevationDeg = Math.toDegrees(Math.atan2(targetZIn, Math.max(1e-6, dist))) * 200 / 26;
            pitchDesired = elevationDegToServoPos(elevationDeg);
        } else {
            pitchDesired = interpPitch(dist);
        }

        pitchCmd = slew(pitchCmd, pitchDesired, pitchSlewPerSec, dt);
        pitchServo.setPosition(pitchCmd);

        boolean targetFresh = targetValid && (System.currentTimeMillis() - lastTargetTime < targetTimeoutMs);

        double yawPower;

        if (targetFresh) {
            // Update yaw target from latest measurement
            yawTargetDeg = targetAngleDeg;

            // Position error (target - measured), wrapped to [-180, 180)
            double yawErrDeg = wrapDeg(yawTargetDeg - yawEstimateDeg);

            // filter to reduce jitter from measurement noise
            double alpha = 0.25;
            filteredYawErrDeg = filteredYawErrDeg + alpha * (yawErrDeg - filteredYawErrDeg);

            tx = filteredYawErrDeg; // keep telemetry variable name
            if (Math.abs(tx) < 0.5) tx = 0.0;

            // Drive yaw error to 0 using encoder feedback
            yawPower = yawPidf.updatePosition(tx, 0, dt);
            out = yawPower;
        } else {
            // If no fresh target, stop yaw
            yawPidf.reset();
            yawPower = 0.0;
        }

        yawPower = Range.clip(yawPower, -yawMaxPower, yawMaxPower);

        // apply inversion
        if (yawInverted) yawPower *= -1.0;

        // Update internal estimate
        if (!hasYawEncoder) {
            yawEstimateDeg += yawPower * yawDegPerSecAtFullPower * dt;
        }

        yawServo.setPower(yawPower);
    }
    public double rawOut() {
        return out;
    }

    public double rawTx() {
        return tx;
    }

    public double rawPos() {
        return yawEstimateDeg;
    }

    public boolean isAimed() {
        boolean targetFresh = targetValid && (System.currentTimeMillis() - lastTargetTime < targetTimeoutMs);
        if (!visionFresh) return false;

        boolean yawOk = Math.abs(visionYawErrorDeg) <= aimTolYawDeg;
        boolean pitchOk = Math.abs(pitchCmd - pitchDesired) <= aimTolPitchPos;
        boolean timeOk = settleTimer.milliseconds() >= settleMs;
        return yawOk && pitchOk && timeOk;
    }

    private double elevationDegToServoPos(double elevationDeg) {
        double minElevation = -20; // degrees down
        double maxElevation = 45;  // degrees up
        elevationDeg = Range.clip(elevationDeg, minElevation, maxElevation);

        double t = (elevationDeg - minElevation) / (maxElevation - minElevation);
        double pos = pitchMinPos + t * (pitchMaxPos - pitchMinPos);
        return Range.clip(pos, pitchMinPos, pitchMaxPos);
    }

    private double interpPitch(double distIn) {
        if (pitchDistIn == null || pitchPos == null || pitchDistIn.length < 2 || pitchDistIn.length != pitchPos.length) {
            return Range.clip(pitchCmd, pitchMinPos, pitchMaxPos);
        }

        if (distIn <= pitchDistIn[0]) return Range.clip(pitchPos[0], pitchMinPos, pitchMaxPos);
        int n = pitchDistIn.length;
        if (distIn >= pitchDistIn[n - 1]) return Range.clip(pitchPos[n - 1], pitchMinPos, pitchMaxPos);

        for (int i = 0; i < n - 1; i++) {
            double d0 = pitchDistIn[i];
            double d1 = pitchDistIn[i + 1];
            if (distIn >= d0 && distIn <= d1) {
                double t = (distIn - d0) / (d1 - d0);
                double p = pitchPos[i] + t * (pitchPos[i + 1] - pitchPos[i]);
                return Range.clip(p, pitchMinPos, pitchMaxPos);
            }
        }
        return Range.clip(pitchPos[n - 1], pitchMinPos, pitchMaxPos);
    }

    private static double slew(double current, double target, double ratePerSec, double dt) {
        double maxStep = Math.abs(ratePerSec) * dt;
        double delta = target - current;
        if (Math.abs(delta) <= maxStep) return target;
        return current + Math.signum(delta) * maxStep;
    }
}

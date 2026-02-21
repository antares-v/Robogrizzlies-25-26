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
    public boolean yawInverted = true;

    // Safety clamp for max yaw power
    public double yawMaxPower = 1.0;
    // Low-pass filter for computed yaw target (deg)
    private double filteredYawTargetDeg = 0.0;

    public boolean useTxForYaw = false;
    public double txSign = 1.0;
    public double txDeadbandDeg = 0.5;

    public double txFilterAlpha = 0.35;
    private double visionTxDeg = 0.0;
    private double filteredTxDeg = 0.0;

    // Approximate turret angular speed (deg/sec)
    public double yawDegPerSecAtFullPower = 178.0;


    // encoder configuration
    // 0-3.3V over one mechanical revolution.
    public double yawEncoderMaxVoltage = 3.3;
    // Turret degrees represented by one full encoder revolution.
    // For 45:132 servo:turret with encoder on servo shaft => 360 * (45/132) = 122.7272.
    public double yawEncoderDegPerRev = 122.7272;
    // Additive offset applied after unwrapping in degrees
    public double yawEncoderOffsetDeg = -75;
    public boolean yawEncoderInverted = false;
    private double yawEncLastRawDeg = 0.0;
    private double yawEncContinuousDeg = 0.0;
    private boolean yawEncHasLast = false;

    // Position PIDF for yaw
    private final CustomPIDF yawPidf;

    // Vision AprilTag measurement in inches in robot space
    private boolean visionValid = false;
    private long lastVisionTime = 0;
    public long visionTimeoutMs = 500;
    private boolean hadVisionLock = false;
    // Mechanical frame offset between "turret zero" and "robot forward".
    // Positive values rotate the target CCW in robot-frame degrees.
    public double yawRobotForwardOffsetDeg = 0.0;

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

    // Target point relative to robot
    private double targetXIn = 0;   // forward
    private double targetYIn = 0;   // left
    private double targetZIn = 0;   // up (optional)

    // Internal pitch state
    private double pitchCmd;
    private double pitchDesired;

    // Internal yaw state (deg)
    private double yawEstimateDeg = 0.0;
    private double yawTargetDeg = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    // debug values
    private double out;
    private double debugYawErrorDeg;

    public TurretController(CRServo yawServo, Servo pitchServo) {
        this(yawServo, pitchServo, null);
    }

    public TurretController(CRServo yawServo, Servo pitchServo, AnalogInput yawEncoder) {
        this.yawServo = yawServo;
        this.pitchServo = pitchServo;

        this.yawEncoder = yawEncoder;
        this.hasYawEncoder = (yawEncoder != null);

        // Position PID defaults (TUNE)
        this.yawPidf = new CustomPIDF(0.01, 0.000000, 0.00003, 0.0);
        this.yawPidf.iMax = 0.2;

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

    public void updateVisionMeasurement(double xIn, double yIn, double zIn, double txDeg, boolean isValid) {
        this.visionValid = isValid;
        if (isValid) {
            this.lastVisionTime = System.currentTimeMillis();
            setTargetRobotRelative(xIn, yIn, zIn);
            this.visionTxDeg = txDeg;
        }
    }

    /** Read the analog yaw encoder and return a turret angle in degrees. */
    private double readYawEncoderDeg() {
        // Voltage -> raw degrees in [0, yawEncoderDegPerRev)
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

        // Update yaw estimate from encoder when available
        if (hasYawEncoder) {
            yawEstimateDeg = readYawEncoderDeg();
        }

        // distance to target
        double dist = Math.sqrt(targetXIn*targetXIn + targetYIn*targetYIn + targetZIn*targetZIn);

        if (Math.abs(targetZIn) > 0.5) {
            double elevationDeg = Math.toDegrees(Math.atan2(targetZIn, Math.max(1e-6, dist))) * 200 / 26;
            pitchDesired = elevationDegToServoPos(elevationDeg);
        } else {
            pitchDesired = interpPitch(dist);
        }

        pitchCmd = slew(pitchCmd, pitchDesired, pitchSlewPerSec, dt);
        pitchServo.setPosition(pitchCmd);

        long now = System.currentTimeMillis();
        boolean visionFresh = visionValid && (now - lastVisionTime < visionTimeoutMs);

        // yaw target
        double desiredYawDegFromPose = Math.toDegrees(Math.atan2(targetYIn, Math.max(1e-6, targetXIn)));

        // filter yaw
        double alphaPose = 0.25;
        filteredYawTargetDeg = filteredYawTargetDeg + alphaPose * (desiredYawDegFromPose - filteredYawTargetDeg);

        // Low-pass filter tx to reduce jitter.
        filteredTxDeg = filteredTxDeg + txFilterAlpha * (visionTxDeg - filteredTxDeg);
        double txUsedDeg = filteredTxDeg;
        if (Math.abs(txUsedDeg) < txDeadbandDeg) txUsedDeg = 0.0;

        double yawPower;
        double rawTargetDeg;

        // Vision tx from a robot-fixed camera should define an absolute target in robot frame,
        // not a delta from current turret angle each loop.
        // When vision is stale, keep aiming using the robot-relative target vector
        // (used for remembered absolute tag tracking).
        if (visionFresh && useTxForYaw) {
            rawTargetDeg = yawRobotForwardOffsetDeg - (txSign * txUsedDeg);
            hadVisionLock = true;
            settleTimer.reset();
        } else {
            rawTargetDeg = filteredYawTargetDeg + yawRobotForwardOffsetDeg;
            if (visionFresh) {
                hadVisionLock = true;
                settleTimer.reset();
            } else if (!hadVisionLock) {
                // If we have never seen vision yet, avoid integrating toward a stale default.
                rawTargetDeg = yawEstimateDeg;
                yawPidf.reset();
            }
        }

        yawTargetDeg = yawEstimateDeg + wrapTo180(rawTargetDeg - yawEstimateDeg);



        yawPower = yawPidf.updatePosition(yawTargetDeg, yawEstimateDeg, dt);
        out = yawPower;
        debugYawErrorDeg = wrapTo180(yawTargetDeg - yawEstimateDeg);

        yawPower = Range.clip(yawPower, -yawMaxPower, yawMaxPower);
        // apply inversion
        if (yawInverted) yawPower *= -1.0;

        // Update internal estimate if we don't have an encoder.
        if (!hasYawEncoder) {
            yawEstimateDeg += yawPower * yawDegPerSecAtFullPower * dt;
        }

        yawServo.setPower(yawPower);
    }

    public double rawOut() {
        return out;
    }

    public double rawYawErrorDeg() {
        return debugYawErrorDeg;
    }

    public double rawPos() {
        return yawEstimateDeg;
    }

    public boolean isAimed() {
        boolean visionFresh = visionValid && (System.currentTimeMillis() - lastVisionTime < visionTimeoutMs);
        if (!visionFresh) return false;

        boolean yawOk = Math.abs(wrapTo180(yawTargetDeg - yawEstimateDeg)) <= aimTolYawDeg;
        boolean pitchOk = Math.abs(pitchCmd - pitchDesired) <= aimTolPitchPos;
        boolean timeOk = settleTimer.milliseconds() >= settleMs;
        return yawOk && pitchOk && timeOk;
    }


    /** Wrap an angle error to (-180, 180] degrees. */
    private static double wrapTo180(double deg) {
        deg = deg % 360.0;
        if (deg <= -180.0) deg += 360.0;
        if (deg > 180.0) deg -= 360.0;
        return deg;
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

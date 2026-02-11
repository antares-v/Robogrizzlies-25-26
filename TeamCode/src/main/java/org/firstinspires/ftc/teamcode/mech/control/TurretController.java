package org.firstinspires.ftc.teamcode.mech.control;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretController {

    // Hardware
    private final CRServo yawServo;
    private final Servo pitchServo;

    // If turret direction is flipped, set true
    public boolean yawInverted = false;

    // Safety clamp for max yaw power
    public double yawMaxPower = 1.0;

    // Approximate turret angular speed (deg/sec) when yawServo is commanded at full power (1.0).
    public double yawDegPerSecAtFullPower = 180.0;

    // Position PIDF for yaw hold/aim (deg -> power)
    private final CustomPIDF yawPidf;

    // Vision AprilTag measurement
    // Positive means the tag is to the right (Limelight targetXDegrees / tx).
    private double visionYawErrorDeg = 0.0;
    private double visionDistanceIn = 0.0;
    private boolean visionValid = false;
    private long lastVisionTime = 0;
    public long visionTimeoutMs = 250;

    // Pitch table (distance in inches to servo position)
    // Must be same length and strictly increasing distances.
    public double[] pitchDistIn = { 18, 30, 42, 54 };
    public double[] pitchPos    = {0.78,0.70,0.64,0.60};

    // Hard clamps for safety
    public double pitchMinPos = 0.45;
    public double pitchMaxPos = 0.90;

    // Slew-rate to prevent pitch oscillations
    public double pitchSlewPerSec = 1.5;

    // Aim tolerance + settle time
    public double aimTolYawDeg = 1.5;
    public double aimTolPitchPos = 0.02;
    public long settleMs = 120;

    // Target point relative to robot (optional; used for pitch if vision distance is not valid)
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

    public TurretController(CRServo yawServo, Servo pitchServo) {
        this.yawServo = yawServo;
        this.pitchServo = pitchServo;

        // Position PID defaults (YOU WILL NEED TO TUNE)
        this.yawPidf = new CustomPIDF(0.020, 0.0, 0.001, 0.0);
        this.yawPidf.iMax = 0.25;

        pitchCmd = pitchServo.getPosition();
        pitchDesired = pitchCmd;

        loopTimer.reset();
        settleTimer.reset();
    }

    /** Zero the internal yaw estimate/target. Call once at init if you want a known reference. */
    public void resetYawEstimate(double yawDeg) {
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
     * Update the AprilTag measurement for aiming.
     * yawErrorDeg: horizontal error angle to tag center in degrees. Positive means tag is to the right.
     * distanceIn: distance to tag (inches) for pitch.
     * isValid: whether a tag was found.
     */
    public void updateVisionMeasurement(double yawErrorDeg, double distanceIn, boolean isValid) {
        this.visionYawErrorDeg = yawErrorDeg;
        this.visionDistanceIn = distanceIn;
        this.visionValid = isValid;
        if (isValid) {
            this.lastVisionTime = System.currentTimeMillis();
        }
    }

    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 1e-6) dt = 0.02;

        // --- Pitch ---
        double dist = Math.hypot(targetXIn, targetYIn);
        if (visionValid) dist = visionDistanceIn;

        if (Math.abs(targetZIn) > 0.5) {
            double elevationDeg = Math.toDegrees(Math.atan2(targetZIn, Math.max(1e-6, dist)));
            pitchDesired = elevationDegToServoPos(elevationDeg);
        } else {
            pitchDesired = interpPitch(dist);
        }

        pitchCmd = slew(pitchCmd, pitchDesired, pitchSlewPerSec, dt);
        pitchServo.setPosition(pitchCmd);

        boolean visionFresh = visionValid && (System.currentTimeMillis() - lastVisionTime < visionTimeoutMs);

        if (visionFresh) {
            double errDeg = yawInverted ? -visionYawErrorDeg : visionYawErrorDeg;
            yawTargetDeg += errDeg;
            settleTimer.reset();
        }

        double yawPower = yawPidf.updatePosition(yawTargetDeg, yawEstimateDeg, dt);
        yawPower = Range.clip(yawPower, -yawMaxPower, yawMaxPower);

        // Update our internal yaw estimate from commanded power.
        yawEstimateDeg += yawPower * yawDegPerSecAtFullPower * dt;

        yawServo.setPower(yawPower);
    }

    public boolean isAimed() {
        boolean visionFresh = visionValid && (System.currentTimeMillis() - lastVisionTime < visionTimeoutMs);
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

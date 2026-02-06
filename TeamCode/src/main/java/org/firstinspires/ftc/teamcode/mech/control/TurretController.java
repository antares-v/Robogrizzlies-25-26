package org.firstinspires.ftc.teamcode.mech.control;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretController {

    // Hardware
    private final CRServo yawServo;
    private final DcMotorEx yawEncoder;
    private final Servo pitchServo;

    public double yawGearRatio = 1.0;

    public double yawEncoderTicksPerRevOverride = -1.0;

    // If turret direction is flipped, set true
    public boolean yawInverted = false;

    // Safety clamp for max yaw power
    public double yawMaxPower = 1.0;

    // Position PIDF for yaw hold/aim
    private final CustomPIDF yawPidf;

    // Vision AprilTag measurement
    // Positive means the tag is to the right if computed with atan2(x,z).
    private double visionYawErrorDeg = 0.0;
    private double visionDistanceIn = 0.0;
    private boolean visionValid = false;
    private long lastVisionTime = 0;
    public long visionTimeoutMs = 250;

    // Pitch table (distance in inches toservo position)
    // Must be same length and strictly increasing distances.
    public double[] pitchDistIn = { 18, 30, 42, 54 };
    public double[] pitchPos    = {0.78,0.70,0.64,0.60};

    // Hard clamps for safety
    public double pitchMinPos = 0.45;
    public double pitchMaxPos = 0.90;

    // Slew limits
    public double pitchSlewPerSec = 1.0;

    // Aim tolerance
    public double aimTolYawDeg = 2.0;
    public double aimTolPitchPos = 0.02;
    public long settleMs = 200;

    // Target (robot-relative)
    private double targetXIn = 24;  // forward
    private double targetYIn = 0;   // left
    private double targetZIn = 0;   // up (optional)

    // Internal state
    private double pitchCmd;
    private double pitchDesired;

    // Yaw position target in encoder ticks
    private double yawTargetTicks = Double.NaN;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    public TurretController(CRServo yawServo, DcMotorEx yawEncoder, Servo pitchServo) {
        this.yawServo = yawServo;
        this.yawEncoder = yawEncoder;
        this.pitchServo = pitchServo;

        // Position PID defaults (YOU WILL NEED TO TUNE)
        this.yawPidf = new CustomPIDF(0.004, 0.0, 0.0002, 0.0);
        this.yawPidf.iMax = 0.25;

        pitchCmd = pitchServo.getPosition();
        pitchDesired = pitchCmd;

        loopTimer.reset();
        settleTimer.reset();
    }

    /** Call once after hardware init if you want to zero the yaw target to current encoder */
    public void resetYawHoldToCurrent() {
        yawTargetTicks = yawEncoder.getCurrentPosition();
        yawPidf.reset();
        settleTimer.reset();
    }

    public void setTargetRobotRelative(double xIn, double yIn, double zIn) {
        this.targetXIn = xIn;
        this.targetYIn = yIn;
        this.targetZIn = zIn;
        settleTimer.reset();
    }

    /**
     * Update the AprilTag measurement for aiming.
     * yawErrorDeg horizontal error angle to tag center in degrees. Positive means tag is to the right.
     * distanceIn distance to tag (inches) for pitch.
     * isValid = whether a tag was found.
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

        // Lazily initialize yaw target
        if (Double.isNaN(yawTargetTicks)) {
            resetYawHoldToCurrent();
        }

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
            yawTargetTicks += errDeg * ticksPerDeg();
            settleTimer.reset();
        }

        double currentTicks = yawEncoder.getCurrentPosition();
        double yawPower = yawPidf.updatePosition(yawTargetTicks, currentTicks, dt);
        yawPower = Range.clip(yawPower, -yawMaxPower, yawMaxPower);

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


    private double encoderTicksPerRev() {
        if (yawEncoderTicksPerRevOverride > 0) return yawEncoderTicksPerRevOverride;
        return yawEncoder.getMotorType().getTicksPerRev();
    }

    private double ticksPerDeg() {
        // ticks/deg = (ticks/rev * gearRatio) / 360
        return (encoderTicksPerRev() * yawGearRatio) / 360.0;
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

    private double slew(double current, double target, double ratePerSec, double dt) {
        double maxStep = ratePerSec * dt;
        double delta = target - current;
        if (Math.abs(delta) <= maxStep) return target;
        return current + Math.signum(delta) * maxStep;
    }

    public void resetYawPID() {
        yawPidf.reset();
        visionValid = false;
    }
}

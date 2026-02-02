package org.firstinspires.ftc.teamcode.mech.control;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.mech.control.CustomPIDF;
public class TurretController {
    private final CRServo yawServo;
    private final Servo pitchServo;
    public double yawMinDeg = -90;      // left limit
    public double yawMaxDeg =  90;      // right limit
    public double yawMinPos = 0.05;     // servo pos at yawMinDeg
    public double yawMaxPos = 0.95;     // servo pos at yawMaxDeg
    public boolean yawInverted = false; // flip if turret turns the wrong way
    private final CustomPIDF yawPidf;
    private double visionYawDegrees = 0;
    private boolean visionValid = false;
    private long lastVisionTime = 0;
    public long visionTimeoutMs = 500;

    // CHANGE THESE VALUES
    // Must be same length and strictly increasing distances.
    public double[] pitchDistIn = { 18, 30, 42, 54 };
    public double[] pitchPos    = {0.78,0.70,0.64,0.60};

    // Hard clamps for safety
    public double pitchMinPos = 0.45;
    public double pitchMaxPos = 0.90;

    public double yawSlewPerSec   = 1.5;   // servo pos units/sec (0..1)
    public double pitchSlewPerSec = 1.0;

    // Aim tolerance
    public double aimTolYawDeg = 2.0;      // good enough tolerance
    public double aimTolPitchPos = 0.02;   // servo pos tolerance
    public long settleMs = 200;            // time to consider settled

    // Target (robot-relative)
    private double targetXIn = 24;  // forward
    private double targetYIn = 0;   // left
    private double targetZIn = 0;   // unused for now

    // Internal state
    private double desiredYawDeg;
    private double pitchCmd = 0.7;

    private double yawDesired = 0.5;
    private double pitchDesired = 0.7;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    public TurretController(CRServo yawServo, Servo pitchServo) {
        this.yawServo = yawServo;
        this.pitchServo = pitchServo;

        // initialize commands to current positions
        // yawCmd = yawServo.getPosition();
        this.yawPidf = new CustomPIDF(0.02, 0.001, 0.002, 0.0); //need tuning
        this.visionYawDegrees = 0;
        this.visionValid = false;
        pitchCmd = pitchServo.getPosition();
//      yawDesired = yawCmd;
        pitchDesired = pitchCmd;

        loopTimer.reset();
        settleTimer.reset();
    }

    public void setTargetRobotRelative(double xIn, double yIn, double zIn) {
        this.targetXIn = xIn;
        this.targetYIn = yIn;
        this.targetZIn = zIn;
        // whenever target changes, restart settle window
        settleTimer.reset();
    }
    public void updateVisionMeasurement(double currentYawDegrees, boolean isValid) {
        this.visionYawDegrees = currentYawDegrees;
        this.visionValid = isValid;
        if (isValid) {
            this.lastVisionTime = System.currentTimeMillis();
        }
    }

    private double elevationDegToServoPos(double elevationDeg) {
        double minElevation = -20; // degrees down
        double maxElevation = 45;  // degrees up

        elevationDeg = Range.clip(elevationDeg, minElevation, maxElevation);

        double t = (elevationDeg - minElevation) / (maxElevation - minElevation);
        double pos = pitchMinPos + t * (pitchMaxPos - pitchMinPos);

        return Range.clip(pos, pitchMinPos, pitchMaxPos);
    }

    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 1e-6) dt = 0.02;

        double dist = Math.hypot(targetXIn, targetYIn);

        desiredYawDeg = Math.toDegrees(Math.atan2(targetYIn, targetXIn)); // existing
        if (Math.abs(targetZIn) > 0.5) {
            double elevationDeg = Math.toDegrees(Math.atan2(targetZIn, dist));
            pitchDesired = elevationDegToServoPos(elevationDeg);
        } else {
            pitchDesired = interpPitch(dist);
        }

        // Check timeout
        boolean visionFresh = visionValid && (System.currentTimeMillis() - lastVisionTime < visionTimeoutMs);

        double yawPower;
        if (visionFresh) {
            yawPower = yawPidf.update(0, visionYawDegrees, dt);
        } else {
            // No vision
            yawPower = 0;
            yawPidf.reset();
        }
        pitchCmd = slew(pitchCmd, pitchDesired, pitchSlewPerSec, dt);

        yawServo.setPower(yawPower);
        pitchServo.setPosition(pitchCmd);
    }
    public boolean isAimed() {
        boolean yawOk;
        if (visionValid) {
            double error = Math.abs(desiredYawDeg - visionYawDegrees);
            // Handle 360 wrap
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            yawOk = Math.abs(error) <= aimTolYawDeg;
        } else {
            yawOk = false; // Can't be aimed if cant be see
        }
        boolean pitchOk = Math.abs(pitchCmd - pitchDesired) <= aimTolPitchPos;
        boolean timeOk = settleTimer.milliseconds() >= settleMs;
        return yawOk && pitchOk && timeOk;
    }
    public void setManual(double yawPos, double pitchPos) {
        yawDesired = Range.clip(yawPos, 0.0, 1.0);
        pitchDesired = Range.clip(pitchPos, 0.0, 1.0);
        settleTimer.reset();
    }

    // helpers

    private double yawDegToServoPos(double yawDeg) {
        yawDeg = Range.clip(yawDeg, yawMinDeg, yawMaxDeg);

        double t = (yawDeg - yawMinDeg) / (yawMaxDeg - yawMinDeg);
        double pos = yawMinPos + t * (yawMaxPos - yawMinPos);

        if (yawInverted) {
            // invert around midpoint
            pos = (yawMinPos + yawMaxPos) - pos;
        }
        return Range.clip(pos, 0.0, 1.0);
    }
    private double servoPosTolForYawDeg(double tolDeg) {
        double rangeDeg = Math.abs(yawMaxDeg - yawMinDeg);
        double rangePos = Math.abs(yawMaxPos - yawMinPos);
        if (rangeDeg < 1e-6) return 0.02;
        return (tolDeg / rangeDeg) * rangePos;
    }

    private double interpPitch(double distIn) {
        if (pitchDistIn == null || pitchPos == null || pitchDistIn.length < 2 || pitchDistIn.length != pitchPos.length) {
            // fallback
            return Range.clip(pitchCmd, pitchMinPos, pitchMaxPos);
        }

        // clamp
        if (distIn <= pitchDistIn[0]) return Range.clip(pitchPos[0], pitchMinPos, pitchMaxPos);
        int n = pitchDistIn.length;
        if (distIn >= pitchDistIn[n - 1]) return Range.clip(pitchPos[n - 1], pitchMinPos, pitchMaxPos);

        // find segment
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

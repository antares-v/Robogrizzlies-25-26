package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Reads ONLY the turret yaw analog encoder and prints telemetry.
 *
 * Controls:
 *  - X: zero yaw (sets current reading to 0 deg)
 */
@TeleOp(name = "Yaw Encoder Only")
public class encoderRead extends LinearOpMode {
    private static final String NAME_TURRET_YAW_ENC = "turretYawEnc";
    private static final double YAW_ENC_MAX_VOLTAGE = 3.3;
    private static final double YAW_ENC_DEG_PER_REV = 122.7272;
    private static final boolean YAW_ENC_INVERTED   = false;

    // Unwrap state
    private double lastRawDeg = 0.0;
    private double continuousDeg = 0.0;
    private boolean hasLast = false;

    // Runtime zero offset
    private double offsetDeg = 0.0;

    @Override
    public void runOpMode() {
        AnalogInput yawEnc;
        try {
            yawEnc = hardwareMap.get(AnalogInput.class, NAME_TURRET_YAW_ENC);
        } catch (Exception e) {
            yawEnc = null;
        }

        telemetry.addLine("Yaw Encoder Only ready.");
        telemetry.addLine("X = zero yaw");
        if (yawEnc == null) telemetry.addLine("ERROR: turretYawEnc not found (check config name).");
        telemetry.update();

        waitForStart();

        boolean xPrev = false;

        while (opModeIsActive()) {
            if (yawEnc == null) {
                telemetry.addLine("turretYawEnc: NOT FOUND");
                telemetry.update();
                idle();
                continue;
            }

            boolean x = gamepad1.x;
            if (x && !xPrev) {
                // Zero: make current continuous reading become 0
                hasLast = false; // restart unwrap cleanly
                double now = readYawContinuousDeg(yawEnc); // this call re-inits continuous
                offsetDeg -= now;
            }
            xPrev = x;

            double v = yawEnc.getVoltage();
            double rawDeg = voltageToRawDeg(v);
            double contDeg = readYawContinuousDeg(yawEnc) + offsetDeg;

            telemetry.addData("Voltage (V)", "%.3f", v);
            telemetry.addData("Raw deg (0..rev)", "%.2f", rawDeg);
            telemetry.addData("Yaw deg (continuous)", "%.2f", contDeg);
            telemetry.update();

            idle();
        }
    }

    private double voltageToRawDeg(double v) {
        double raw = (v / Math.max(1e-6, YAW_ENC_MAX_VOLTAGE)) * YAW_ENC_DEG_PER_REV;

        raw = raw % YAW_ENC_DEG_PER_REV;
        if (raw < 0) raw += YAW_ENC_DEG_PER_REV;

        if (YAW_ENC_INVERTED) {
            raw = YAW_ENC_DEG_PER_REV - raw;
            if (raw >= YAW_ENC_DEG_PER_REV) raw -= YAW_ENC_DEG_PER_REV;
        }
        return raw;
    }

    private double readYawContinuousDeg(AnalogInput enc) {
        double raw = voltageToRawDeg(enc.getVoltage());

        if (!hasLast) {
            hasLast = true;
            lastRawDeg = raw;
            continuousDeg = raw;
        } else {
            double delta = raw - lastRawDeg;
            double half = YAW_ENC_DEG_PER_REV / 2.0;

            if (delta > half)  delta -= YAW_ENC_DEG_PER_REV;
            if (delta < -half) delta += YAW_ENC_DEG_PER_REV;

            continuousDeg += delta;
            lastRawDeg = raw;
        }

        return continuousDeg;
    }
}

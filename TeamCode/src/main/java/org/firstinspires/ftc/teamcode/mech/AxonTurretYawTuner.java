package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * AxonTurretYawTuner
 *
 * Designed for a continuous-rotation Axon servo (CRServo) + analog absolute encoder (AnalogInput).
 * Shows encoder voltage, angle (deg/rad), wrap events, and lets you measure degrees-per-output-rev.
 *
 * Controls:
 *  - Left stick Y: manual servo power (up = positive)
 *  - A: capture START (voltage + wrap counter)
 *  - B: capture END   (voltage + wrap counter) and compute delta over revolutions
 *  - X: reset capture + revolution counter
 *
 * IMPORTANT:
 *  - Analog absolute encoders don't have "ticks per revolution" like quadrature encoders.
 *    They output a voltage (typically 0-3.3V) that wraps once per encoder-shaft revolution.
 *  - If you truly need "ticks", that only exists for digital encoders. For analog, you use degrees/volt.
 */
@TeleOp(name = "Axon Turret Yaw Tuner", group = "Debug")
public class AxonTurretYawTuner extends LinearOpMode {

    private static final String SERVO_NAME = "yawServo";    // TODO: CRServo device name in RC config
    private static final String ANALOG_NAME = "yawEncoder"; // TODO: AnalogInput device name in RC config

    // Encoder electrical range (REV Analog is typically 0-3.3V)
    private static final double ENCODER_MAX_V = 3.3;

    // Servo:Turret = 45:132 (servo rev produces 45/132 turret rev)
    // - If encoder is on SERVO shaft and you want turret output angle, keep 45/132.
    // - If encoder is on TURRET/output, set this to 1.0.
    private static final double OUTPUT_REV_PER_ENCODER_REV = 45.0 / 132.0; // TODO: adjust if needed

    private CRServo yawServo;
    private AnalogInput yawEncoder;

    // Wrap-aware tracking
    private double lastV = 0.0;
    private int wrapCount = 0;
    private boolean trackerInitialized = false;

    // Capture points
    private boolean startCaptured = false;
    private boolean endCaptured = false;
    private double startV = 0.0, endV = 0.0;
    private int startWrap = 0, endWrap = 0;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        yawServo = hardwareMap.get(CRServo.class, SERVO_NAME);
        yawEncoder = hardwareMap.get(AnalogInput.class, ANALOG_NAME);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Axon Turret Yaw Tuner");
        telemetry.addLine("LS Y: servo power | A: start | B: end | X: reset");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Manual servo control
            double power = -gamepad1.left_stick_y; // up is negative; invert so up = +
            power = clip(power, -1.0, 1.0);
            yawServo.setPower(power);

            // Read encoder voltage
            double v = yawEncoder.getVoltage();

            // Initialize wrap tracker
            if (!trackerInitialized) {
                lastV = v;
                wrapCount = 0;
                trackerInitialized = true;
            } else {
                updateWrapCounter(lastV, v);
                lastV = v;
            }

            // Buttons
            if (gamepad1.x) {
                startCaptured = false;
                endCaptured = false;
                wrapCount = 0;
                trackerInitialized = true;
                lastV = v;
                sleep(200);
            }

            if (gamepad1.a && !startCaptured) {
                startV = v;
                startWrap = wrapCount;
                startCaptured = true;
                endCaptured = false;
                sleep(200);
            }

            if (gamepad1.b && startCaptured && !endCaptured) {
                endV = v;
                endWrap = wrapCount;
                endCaptured = true;
                sleep(200);
            }

            // Continuous encoder revolutions since last reset:
            double encRevNow = wrapCount + (v / ENCODER_MAX_V);
            double encDegNow = encRevNow * 360.0;

            // Convert to turret/output revolutions (if encoder is on servo shaft, this accounts for 45:132)
            double outRevNow = encRevNow * OUTPUT_REV_PER_ENCODER_REV;
            double outDegNow = outRevNow * 360.0;

            telemetry.addData("Uptime (s)", "%.1f", runtime.seconds());
            telemetry.addLine("\n--- Live ---");
            telemetry.addData("Servo power", "%.2f", power);
            telemetry.addData("Voltage", "%.4f V", v);
            telemetry.addData("Wrap count", wrapCount);
            telemetry.addData("Encoder rev (est)", "%.4f", encRevNow);
            telemetry.addData("Encoder deg (est)", "%.1f", encDegNow);
            telemetry.addData("Output rev (est)", "%.4f", outRevNow);
            telemetry.addData("Output deg (est)", "%.1f", outDegNow);
            telemetry.addData("OUTPUT_REV_PER_ENCODER_REV", "%.6f", OUTPUT_REV_PER_ENCODER_REV);

            telemetry.addLine("\n--- Capture ---");
            telemetry.addLine("A=start  B=end  X=reset");
            if (startCaptured) telemetry.addData("Start", "V=%.4f wrap=%d", startV, startWrap);
            if (endCaptured) telemetry.addData("End", "V=%.4f wrap=%d", endV, endWrap);

            if (startCaptured && endCaptured) {
                double startEncRev = startWrap + (startV / ENCODER_MAX_V);
                double endEncRev = endWrap + (endV / ENCODER_MAX_V);

                double deltaEncRev = endEncRev - startEncRev;
                double deltaOutRev = deltaEncRev * OUTPUT_REV_PER_ENCODER_REV;
                double deltaOutDeg = deltaOutRev * 360.0;

                telemetry.addLine("\n--- Result ---");
                telemetry.addData("Delta encoder rev", "%.4f", deltaEncRev);
                telemetry.addData("Delta output rev", "%.4f", deltaOutRev);
                telemetry.addData("Delta output deg", "%.1f", deltaOutDeg);
                telemetry.addLine("Rotate N OUTPUT revs between A and B, then:");
                telemetry.addData("Output rev per captured", "%.4f", deltaOutRev);
                telemetry.addData("Deg per output rev (sanity)", "%.2f", (Math.abs(deltaOutRev) > 1e-6) ? (Math.abs(deltaOutDeg) / Math.abs(deltaOutRev)) : 0.0);
                telemetry.addLine("Ideal deg/outputRev ~ 360. Use multiple revs for accuracy.");
            }

            telemetry.update();
        }
    }


    private void updateWrapCounter(double prevV, double newV) {
        double half = ENCODER_MAX_V / 2.0;
        double dv = newV - prevV;

        // Jumped downward a lot => wrapped forward past max -> 0
        if (dv < -half) {
            wrapCount += 1;
        }
        // Jumped upward a lot => wrapped backward past 0 -> max
        else if (dv > half) {
            wrapCount -= 1;
        }
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

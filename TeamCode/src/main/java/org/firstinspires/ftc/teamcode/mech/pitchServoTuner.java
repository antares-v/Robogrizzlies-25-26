package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Pitch Servo Tuner:
 * - Shows servo position (0..1) and an estimated pitch angle (deg) using a linear map.
 * - D-pad UP/DOWN nudges the servo slightly.
 *
 * IMPORTANT:
 * A standard positional servo does NOT know its real angle. The "angle" shown here is an estimate
 * based on your calibration constants below.
 */
@TeleOp(name = "Pitch Servo Angle Viewer (Tuner)", group = "Tuning")
public class pitchServoTuner extends LinearOpMode {

    // ----------- CONFIGURE THESE -----------
    private static final String PITCH_SERVO_NAME = "turretPitch"; // <-- change to your hardware name

    // Safe mechanical range for your linkage (NOT necessarily 0..1).
    private static final double PITCH_MIN_POS = 0.15;  // <-- find by testing (just before hard stop)
    private static final double PITCH_MAX_POS = 0.85;  // <-- find by testing (just before hard stop)

    // Angle calibration for linear mapping:
    // Define what angle (degrees) your mechanism is at when the servo is at MIN and MAX.
    private static final double MIN_ANGLE_DEG = 0.0;   // angle when servo is at PITCH_MIN_POS
    private static final double MAX_ANGLE_DEG = 60.0;  // angle when servo is at PITCH_MAX_POS

    // Nudge size per "click" of d-pad (smaller = finer control).
    private static final double NUDGE_STEP_POS = 0.05;

    // Debounce so holding d-pad doesn’t spam too fast. (seconds)
    private static final double NUDGE_COOLDOWN_S = 0.08;
    // --------------------------------------

    private Servo pitchServo;
    private double currentPos;

    private final ElapsedTime nudgeTimer = new ElapsedTime();
    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void runOpMode() {
        pitchServo = hardwareMap.get(Servo.class, PITCH_SERVO_NAME);

        // Start at midpoint of your safe range
        currentPos = (PITCH_MIN_POS + PITCH_MAX_POS) / 2.0;
        setPitchPos(currentPos);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Pitch Servo Tuner Ready");
        telemetry.addLine("D-pad UP/DOWN: nudge pitch");
        telemetry.addLine("Edit constants at top for your servo name + calibration.");
        telemetry.update();

        waitForStart();
        nudgeTimer.reset();

        while (opModeIsActive()) {
            // Read inputs
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            // Edge-triggered nudges with cooldown
            if (nudgeTimer.seconds() >= NUDGE_COOLDOWN_S) {
                if (up && !lastUp) {
                    currentPos += NUDGE_STEP_POS;
                    setPitchPos(currentPos);
                    nudgeTimer.reset();
                } else if (down && !lastDown) {
                    currentPos -= NUDGE_STEP_POS;
                    setPitchPos(currentPos);
                    nudgeTimer.reset();
                }
            }

            lastUp = up;
            lastDown = down;

            // Display telemetry
            double clippedPos = Range.clip(currentPos, PITCH_MIN_POS, PITCH_MAX_POS);
            double estAngleDeg = posToAngleDeg(clippedPos);

            telemetry.addData("Servo Name", PITCH_SERVO_NAME);
            telemetry.addData("Servo Pos (0..1)", "%.4f", clippedPos);
            telemetry.addData("Estimated Angle (deg)", "%.2f", estAngleDeg);
            telemetry.addData("Min/Max Pos", "%.3f / %.3f", PITCH_MIN_POS, PITCH_MAX_POS);
            telemetry.addData("Min/Max Angle", "%.1f / %.1f", MIN_ANGLE_DEG, MAX_ANGLE_DEG);
            telemetry.addLine("Tip: If angle moves the wrong way, swap MIN/MAX angles or invert your servo direction.");
            telemetry.update();

            idle();
        }
    }

    private void setPitchPos(double pos) {
        double clipped = Range.clip(pos, PITCH_MIN_POS, PITCH_MAX_POS);
        currentPos = clipped;
        pitchServo.setPosition(clipped);
    }

    /** Linear mapping: servo position -> estimated mechanism angle (deg). */
    private double posToAngleDeg(double pos) {
        // Normalize within [PITCH_MIN_POS, PITCH_MAX_POS]
        double t = (pos - PITCH_MIN_POS) / Math.max(1e-9, (PITCH_MAX_POS - PITCH_MIN_POS));
        t = Range.clip(t, 0.0, 1.0);
        return MIN_ANGLE_DEG + t * (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
    }

    /** Optional if you ever want to command an angle and convert to a position. */
    @SuppressWarnings("unused")
    private double angleDegToPos(double angleDeg) {
        double t = (angleDeg - MIN_ANGLE_DEG) / Math.max(1e-9, (MAX_ANGLE_DEG - MIN_ANGLE_DEG));
        t = Range.clip(t, 0.0, 1.0);
        return PITCH_MIN_POS + t * (PITCH_MAX_POS - PITCH_MIN_POS);
    }
}
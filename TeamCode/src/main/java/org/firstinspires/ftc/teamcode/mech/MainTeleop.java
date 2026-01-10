package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mech.CV.ColorDetection;
import org.firstinspires.ftc.teamcode.mech.movement.movement;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


@TeleOp
public class MainTeleop extends LinearOpMode {

    // Hardware maps
    private movement drive;
    private CRServo leftFlywheel, rightFlywheel;
    private RevColorSensorV3 sensor;
    private Servo spindexer;
    private DcMotorEx leftIntake, rightIntake, launcher;

    // Spindexer positions
    private final double[] spindexerPosIntake  = {0.00, 0.38, 0.79};
    private final double[] spindexerPosOuttake = {0.19, 0.59, 0.99};

    // Ball tracking
    private final List<String> ballcols = new ArrayList<>();
    private final ColorDetection colorSensor = new ColorDetection();
    private int i = 0;
    private final ElapsedTime spintime = new ElapsedTime();

    // Button tracking
    private boolean dLeftPrev = false, dRightPrev = false;
    private boolean xPrev = false, yPrev = false, bPrev = false, aPrev = false;

    // Pattern selection
    private boolean patternChecked = false;
    private int p = 0;                    // where green should end up (0/1/2)
    private String patternName = "random";
    private String stype = "none";
    // Shooter states
    private enum ShootState { IDLE, SET_SERVO, SPINUP, FIRE, RECOVER }
    private ShootState shootState = ShootState.IDLE; //initial shootstate

    private final ElapsedTime shootTimer = new ElapsedTime();
    private int[] shotOrder = new int[] {0, 1, 2}; // indices into spindexerPosOuttake
    private int shotIndex = 0;

    // Timing knobs (ms)
    private static final long FIRST_SPINUP_MS = 1700;
    private static final long NEXT_SPINUP_MS  = 700;
    private static final long FIRE_MS         = 1000;
    private static final long RECOVER_MS      = 220;

    private static final float LAUNCH_POWER = 0.1f;

    boolean rotated = false;

    boolean outtaking = false;
    // Helpers
    private static double deadzone(double v, double dz) {
        return (Math.abs(v) < dz) ? 0 : v;
    }

    @Override
    public void runOpMode() {
        // Init
        drive = new movement(this, 0, 0, 0);

        leftFlywheel = hardwareMap.get(CRServo.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(CRServo.class, "rightFlywheel");
        spindexer = hardwareMap.get(Servo.class, "spindexer");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");
        sensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        rightIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // init ball list
        ballcols.clear();
        ballcols.add("blank");
        ballcols.add("blank");
        ballcols.add("blank");

        spintime.reset();
        shootTimer.reset();
        stopShooter();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        colorSensor.enableLed(sensor);
        // main loop
        while (opModeIsActive()) {
            // 1) Drive
            double x = deadzone(gamepad1.left_stick_x, 0.2);   // strafe
            double y = deadzone(gamepad1.left_stick_y, 0.2);  // forward
            double h = deadzone(gamepad1.right_stick_x, 0.2);  // turn
            drive.move(x, y, h);

            // 2) Edge detection
            boolean dLeft = gamepad1.dpad_left;
            boolean dRight = gamepad1.dpad_right;

            boolean dLeftPressed  = dLeft  && !dLeftPrev;
            boolean dRightPressed = dRight && !dRightPrev;

            dLeftPrev = dLeft;
            dRightPrev = dRight;

            boolean xNow = gamepad1.x;
            boolean yNow = gamepad1.y;
            boolean bNow = gamepad1.b;
            boolean aNow = gamepad1.a;

            boolean xPressed = xNow && !xPrev;
            boolean yPressed = yNow && !yPrev;
            boolean bPressed = bNow && !bPrev;
            boolean aPressed = aNow && !aPrev;

            xPrev = xNow;
            yPrev = yNow;
            bPrev = bNow;
            aPrev = aNow;


            // 3) Intake
            boolean lB = gamepad1.left_bumper;
            boolean rB = gamepad1.right_bumper;

            if (rB) {
                rightIntake.setVelocity(1);
                leftIntake.setVelocity(1);
                spindexer.setPosition(spindexerPosIntake[i]);
            } else if (lB) {
                rightIntake.setVelocity(-1);
                leftIntake.setVelocity(-1);
                spindexer.setPosition(spindexerPosIntake[i]);
            } else {
                rightIntake.setVelocity(0);
                leftIntake.setVelocity(0);
            }
            if (!rotated && !outtaking) {
                ballcols.set(i, colorSensor.getColor(sensor));
            }

            if (spintime.milliseconds() > 100 && rotated) {
                rotated = false;
            }

            // 4) Indexer and sample color
            if (dLeftPressed && i < spindexerPosIntake.length - 1 && !rotated) {
                rotated = true;
                i++;
                spintime.reset();
                spindexer.setPosition(spindexerPosIntake[i]);
                telemetry.update();
            }
            if (dRightPressed && i > 0 && !rotated) {
                rotated = true;
                outtaking = false;
                i--;
                spintime.reset();
                spindexer.setPosition(spindexerPosIntake[i]);
                telemetry.update();
            }

            // 5) Pattern selection (one-time at the start or round) (X, Y, B)
            boolean consumedYThisLoop = false;

            if (!patternChecked) {
                if (xPressed) {
                    p = 0;
                    patternChecked = true;
                    patternName = "g_first";
                } else if (yPressed) {
                    p = 1;
                    patternChecked = true;
                    patternName = "g_second";
                    consumedYThisLoop = true; // don't fire on same press
                } else if (bPressed) {
                    p = 2;
                    patternChecked = true;
                    patternName = "g_third";
                }
            }

            // 6) Cancel firing immediately for fallback (A)
            if (aPressed) {
                cancelShooting();
            }

            // 7) Start firing (Y)
            if (patternChecked && yPressed && !consumedYThisLoop && shootState == ShootState.IDLE) {
                int[] order = computeShotOrder(ballcols, p);
                stype = Arrays.toString(order);
                telemetry.update();
                startShooting(order);
            }

            // 8) Update shooter states
            updateShooting();

            // Telemetry updates
            telemetry.addData("drive", "x=%.2f y=%.2f h=%.2f", x, y, h);
            telemetry.addData("pattern", patternName);
            telemetry.addData("colors", ballcols);
            telemetry.addData("i", i);
            telemetry.addData("shootState", shootState);
            telemetry.addData("shotIndex", shotIndex);
            telemetry.addData("typeofshot", stype);
            telemetry.update();

            idle();
        }
    }

    // Builds the order array,
    // fallback to [0,1,2]
    private int[] computeShotOrder(List<String> ballcols, int p) {
        // Default fallback
        int[] fallback = new int[] {0, 1, 2};

        List<Integer> poslist = new ArrayList<>();
        poslist.add(0); poslist.add(0); poslist.add(0);

        boolean greenFound = false;
        int purple = 0;

        for (int j = 0; j < 3; j++) {
            String color = ballcols.get(j);

            if ("green".equals(color) && !greenFound) {
                poslist.set(p, j);
                poslist.set((p + 1) % 3, (j + 1) % 3);
                poslist.set((p + 2) % 3, (j + 2) % 3);
                greenFound = true;
                break;
            } else if (("green".equals(color) && greenFound)) {
                // randomizer condition
                return fallback;
            } else if ("purple".equals(color)) {
                purple++;
                if (purple > 2) return fallback; // reject 3 purples
            } else {
                return fallback; // unknown color string
            }
        }
        if (!greenFound) return fallback;
        // Convert to int[]
        int[] order = new int[3];
        for (int k = 0; k < 3; k++) order[k] = poslist.get(k);
        return order;
    }

    // Shooting States
    private void startShooting(int[] order) {
        shotOrder = order.clone();
        shotIndex = 0;

        // kick off
        shootState = ShootState.SET_SERVO;
        shootTimer.reset();
    }

    private void cancelShooting() {
        shootState = ShootState.IDLE;
        shotIndex = 0;
        stopShooter();
    }

    private void updateShooting() {
        //basically a switch system each switch triggers the next one with cases.
        switch (shootState) {
            case IDLE:
                outtaking = false;
                return;

            case SET_SERVO: {
                outtaking = true;
                // Move servo to the next desired outtake position
                int posIdx = shotOrder[shotIndex];

                posIdx = (posIdx + 1) % 3; // Spindexer Outtake is offsetted by 1.

                posIdx = Math.max(0, Math.min(posIdx, spindexerPosOuttake.length - 1));

                spindexer.setPosition(spindexerPosOuttake[posIdx]);

                // Start launcher motor
                launcher.setVelocity(LAUNCH_POWER);

                shootTimer.reset();
                shootState = ShootState.SPINUP;
                break;
            }

            case SPINUP: {
                long needed = (shotIndex == 0) ? FIRST_SPINUP_MS : NEXT_SPINUP_MS;
                if (shootTimer.milliseconds() >= needed) {
                    // Flywheels on
                    leftFlywheel.setPower(1);
                    rightFlywheel.setPower(1);

                    shootTimer.reset();
                    shootState = ShootState.FIRE;
                }
                break;
            }

            case FIRE: {
                if (shootTimer.milliseconds() >= FIRE_MS) {
                    // Stop everything for a moment (matches your old behavior)
                    leftFlywheel.setPower(0);
                    rightFlywheel.setPower(0);
                    shootTimer.reset();
                    shootState = ShootState.RECOVER;
                }
                break;
            }

            case RECOVER: {
                if (shootTimer.milliseconds() >= RECOVER_MS) {
                    shotIndex++;

                    if (shotIndex >= shotOrder.length) {
                        shootState = ShootState.IDLE;
                        stopShooter();
                    } else {
                        shootState = ShootState.SET_SERVO;
                    }
                    outtaking = false;
                }
                break;
            }
        }
    }

    private void stopShooter() {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
        launcher.setVelocity(0);
    }
}

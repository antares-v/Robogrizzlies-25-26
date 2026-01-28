package org.firstinspires.ftc.teamcode.mech;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.mech.Auto.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.mech.CV.ColorDetection;
import org.firstinspires.ftc.teamcode.mech.movement.movement;
import org.firstinspires.ftc.teamcode.mech.control.CustomPIDF;
import org.firstinspires.ftc.teamcode.mech.control.TurretController;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


@TeleOp
public class MainTeleop extends LinearOpMode {

    // Hardware maps
    private movement drive;
    private CRServo bottomFlywheel, topFlywheel;
    private RevColorSensorV3 sensor;
    // private Servo spindexer;  // SPINDEXER DISABLED
    private DcMotorEx backIntake, frontIntake, launcher;

    // Spindexer positions
    // private final double[] spindexerPosIntake  = {0.00, 0.38, 0.79};  // SPINDEXER DISABLED
    // private final double[] spindexerPosOuttake = {0.19, 0.59, 0.99};  // SPINDEXER DISABLED

    // Ball tracking
    private final List<String> ballcols = new ArrayList<>();
    private final ColorDetection colorSensor = new ColorDetection();
    private int i = 0;
    private final ElapsedTime spintime = new ElapsedTime();

    // Button tracking
    private boolean dLeftPrev = false, dRightPrev = false;
    private boolean xPrev = false, yPrev = false, bPrev = false, aPrev = false;

    // Pattern selection
    private boolean patternChecked = true;  // order sorting disabled
    private int p = 0;                    // where green should end up (0/1/2)
    private String patternName = "random";
    private String stype = "none";

    // Auto-index
    private boolean autoIndexLockout = false;          // true = don't auto-move spindexer
    private boolean autoIndexedSinceLastBall = false;  // true = auto moved since last ball was detected

    // Shooter states
    private enum ShootState { IDLE, AIM, SET_SERVO, SPINUP, FIRE, RECOVER }
    private ShootState shootState = ShootState.IDLE; //initial shootstate

    private final ElapsedTime shootTimer = new ElapsedTime();
    // Single-shot robot (no spindexer / no multi-shot sequencing)
    private int[] shotOrder = new int[] {0};
    private int shotIndex = 0;

    // Timing knobs (ms)
    private static final long FIRST_SPINUP_MS = 3000;
    private static final long NEXT_SPINUP_MS  = 700;
    private static final long FIRE_MS         = 1000;
    private static final long RECOVER_MS      = 5000;
    // Launcher encoder/velocity tuning
    private static final double LAUNCHER_TICKS_PER_REV = 28.0;
    // target RPMs (tune these)
    private static final double TARGET_RPM_FIRST = 500.0;
    private static final double TARGET_RPM_NEXT  = 600.0;

    // Battery + launcher velocity compensation
    private static final double NOMINAL_VOLTAGE = 12.0;
    private PIDFCoefficients baseLauncherPIDF;
    private double launcherTicksPerRev;

    // "At speed" logic
    private final ElapsedTime rpmStableTimer = new ElapsedTime();
    private static final long STABLE_MS = 100;           // must be at speed this long before feeding
    private static final double RPM_TOL_FRAC = 0.05;     // +/-3% window around target

    // Feeder behavior (CRServos that push ball into launcher)
    private static final double FEED_POWER = 1.0;       // tune (0.6–1.0)
    private static final long FEED_MS = 500;

    // computed velocity targets (ticks per second)
    private final double TARGET_VEL_FIRST = TARGET_RPM_FIRST * LAUNCHER_TICKS_PER_REV / 60.0;
    private final double TARGET_VEL_NEXT  = TARGET_RPM_NEXT  * LAUNCHER_TICKS_PER_REV / 60.0;

    // when this fraction of target is reached we consider it spun up
    private static final double VEL_THRESHOLD_FRAC = 0.90;

    // runtime fields
    private double currentTargetVel = 0.0;
    boolean rotated = false;

    boolean outtaking = false;

    // launcher velocity control
    private CustomPIDF launcherPIDF;
    private final ElapsedTime launcherLoopTimer = new ElapsedTime();
    private double launcherTargetTicksPerSec = 0.0;
    private boolean launcherControlEnabled = false;

    // turret control

    private Servo turretYaw, turretPitch;
    private TurretController turret;

    private PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, 0.00199746322, new Pose2d(0, 0, Math.toRadians(90)));
    private Pose2d robotPos;

    // tune values
    private static double LAUNCH_kP = 0.00025;
    private static double LAUNCH_kI = 0.0000008;
    private static double LAUNCH_kD = 0.00001;

    // kF will be computed from motor max speed at init, but you can override if you want:
    private static double LAUNCH_kF = -1.0; // -1 = auto compute


    // Helpers
    private static double deadzone(double v, double dz) {
        return (Math.abs(v) < dz) ? 0 : v;
    }

    @Override
    public void runOpMode() {
        // Init
        drive = new movement(this, 0, 0, 0);

        bottomFlywheel = hardwareMap.get(CRServo.class, "bottomFlywheel");
        topFlywheel = hardwareMap.get(CRServo.class, "topFlywheel");
        // spindexer = hardwareMap.get(Servo.class, "spindexer");  // SPINDEXER DISABLED
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");
        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        sensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        turretYaw   = hardwareMap.get(Servo.class, "turretYaw");
        turretPitch = hardwareMap.get(Servo.class, "turretPitch");

        turret = new TurretController(turretYaw, turretPitch);

        turret.setTargetRobotRelative(36, 10, 0);

        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        frontIntake.setDirection(DcMotorEx.Direction.REVERSE);

        bottomFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        topFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

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

        launcherTicksPerRev = launcher.getMotorType().getTicksPerRev();
        baseLauncherPIDF = launcher.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Max ticks/sec = maxRPM * ticksPerRev / 60
        double maxRpm = launcher.getMotorType().getMaxRPM();
        double maxTicksPerSec = (maxRpm * launcherTicksPerRev) / 60.0;

        double kF = (LAUNCH_kF > 0) ? LAUNCH_kF : (1.0 / maxTicksPerSec);

        launcherPIDF = new CustomPIDF(LAUNCH_kP, LAUNCH_kI, LAUNCH_kD, kF);
        launcherPIDF.iMax = 0.35; // clamp integral contribution (power units)
        launcherPIDF.reset();

        launcherLoopTimer.reset();
        launcherControlEnabled = false;
        launcherTargetTicksPerSec = 0.0;


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
                // spindexer.setPosition(spindexerPosIntake[i]);  // SPINDEXER DISABLED
                frontIntake.setPower(1);
                backIntake.setPower(1);
            } else if (lB) {
                // spindexer.setPosition(spindexerPosIntake[i]);  // SPINDEXER DISABLED
                frontIntake.setPower(-1);
                backIntake.setPower(-1);
            } else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            }

            // AUTO-INDEXING (SPINDEXER) DISABLED
            // Only do auto-indexing when not shooting
            // if (!rotated && shootState == ShootState.IDLE && !outtaking) {
                // String prev = ballcols.get(i);
                // String now  = colorSensor.getColor(sensor);
                // ballcols.set(i, now);
// 
                // boolean newBallArrived = prev.equals("blank") && !now.equals("blank");
                // if (newBallArrived) {
                    // Re-enable auto-indexing after a new ball is intaken/detected
                    // autoIndexLockout = false;
                    // autoIndexedSinceLastBall = false;
                // }
// 
                // if (!autoIndexLockout && !ballcols.get(i).equals("blank")) {
                    // for (int j = 0; j < 3; j++) {
                        // if (ballcols.get(j).equals("blank")) {
                            // i = j;
                            // spindexer.setPosition(spindexerPosIntake[i]);  // SPINDEXER DISABLED
                            // spintime.reset();
                            // rotated = true;
// 
                            // autoIndexedSinceLastBall = true; // remember that auto moved
                            // break;
                        // }
                    // }
                // }
            // }


            if (spintime.milliseconds() > 100 && rotated) {
                rotated = false;
            }

            // MANUAL INDEXING (SPINDEXER) DISABLED
            // 4) Indexer and sample color
            // if (dLeftPressed && i < spindexerPosIntake.length - 1 && !rotated) {
                // rotated = true;
                // outtaking = false;
                // i++;
                // spintime.reset();
                // spindexer.setPosition(spindexerPosIntake[i]);  // SPINDEXER DISABLED
// 
                // if (autoIndexedSinceLastBall) autoIndexLockout = true; // driver override after auto
                // telemetry.update();
            // }
// 
            // if (dRightPressed && i > 0 && !rotated) {
                // rotated = true;
                // outtaking = false;
                // i--;
                // spintime.reset();
                // spindexer.setPosition(spindexerPosIntake[i]);  // SPINDEXER DISABLED
// 
                // if (autoIndexedSinceLastBall) autoIndexLockout = true; // driver override after auto
                // telemetry.update();
            // }
// 
            // PATTERN / ORDER SORTING DISABLED (NO SPINDEXER)
            // 5) Pattern selection (one-time at the start or round) (X, Y, B)
            // boolean consumedYThisLoop = false;
// 
            // if (!patternChecked) {
                // if (xPressed) {
                    // p = 0;
                    // patternChecked = true;
                    // patternName = "g_first";
                // } else if (yPressed) {
                    // p = 1;
                    // patternChecked = true;
                    // patternName = "g_second";
                    // consumedYThisLoop = true; // don't fire on same press
                // } else if (bPressed) {
                    // p = 2;
                    // patternChecked = true;
                    // patternName = "g_third";
                // }
            // }
// 
            // 6) Cancel firing immediately for fallback (A)
            if (aPressed) {
                cancelShooting();
            }

            // 7) Start firing (Y)
            if (yPressed && shootState == ShootState.IDLE) {
                // No spindexer / no sorting / single-ball robot: spin up launcher, then feed once.
                stype = "single";
                telemetry.update();
                startShooting(new int[]{0});
            }

            // 8) Update shooter states
            updateShooting();

            // 9) Update PIDF
            updateLauncherPIDF();

            // 10) Update localizer
            localizer.update();

            // 11) Update turret
            robotPos = localizer.getPose();
            turret.setTargetRobotRelative(robotPos.position.x, robotPos.position.y, 0);
            turret.update();

            // Telemetry updates
            telemetry.addData("drive", "x=%.2f y=%.2f h=%.2f", x, y, h);
            telemetry.addData("pattern", patternName);
            telemetry.addData("colors", ballcols);
            telemetry.addData("i", i);
            telemetry.addData("shootState", shootState);
            telemetry.addData("shotIndex", shotIndex);
            telemetry.addData("typeofshot", stype);
            telemetry.addData("pos", launcher.getCurrentPosition());
            telemetry.addData("vel", launcher.getVelocity());
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
        shootState = ShootState.AIM;
        shootTimer.reset();
    }

    private void cancelShooting() {
        shootState = ShootState.IDLE;
        shotIndex = 0;
        stopShooter();
    }

    private double batteryVoltage() {
        double minV = 99.0;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0) minV = Math.min(minV, v);
        }
        return (minV < 99.0) ? minV : NOMINAL_VOLTAGE;
    }

    // Voltage compensation for CRServo power (keeps feed speed more consistent)
    private double vcPower(double pwr) {
        double scale = NOMINAL_VOLTAGE / batteryVoltage();
        return Range.clip(pwr * scale, -1.0, 1.0);
    }

    // Compensate launcher F term so velocity loop behaves similarly as voltage changes
    private void applyLauncherVoltageCompToF() {
        double scale = NOMINAL_VOLTAGE / batteryVoltage();
        launcher.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(baseLauncherPIDF.p, baseLauncherPIDF.i, baseLauncherPIDF.d, baseLauncherPIDF.f * scale)
        );
    }

    private void setLauncherRPM(double rpm) {
        launcherTargetTicksPerSec = rpm * launcherTicksPerRev / 60.0;
        launcherControlEnabled = (rpm > 0.0);
        launcherPIDF.reset();
        launcherLoopTimer.reset();
    }


    private double getLauncherRPM() {
        return launcher.getVelocity() / launcherTicksPerRev * 60.0;
    }

    private boolean launcherAtSpeed(double targetRpm) {
        double rpm = getLauncherRPM();
        double err = Math.abs(rpm - targetRpm);
        return err <= (RPM_TOL_FRAC * targetRpm);
    }
    private void updateLauncherPIDF() {
        if (!launcherControlEnabled) return;

        double dt = launcherLoopTimer.seconds();
        launcherLoopTimer.reset();

        double measured = launcher.getVelocity(); // ticks/sec
        double target = launcherTargetTicksPerSec;

        double power = launcherPIDF.update(target, measured, dt);

        // Optional: voltage compensation (helps keep behavior consistent)
        double scale = NOMINAL_VOLTAGE / batteryVoltage();
        power = Range.clip(power * scale, -1.0, 1.0);

        launcher.setPower(power);

        telemetry.addData("L PID", "t=%.0f m=%.0f pwr=%.2f dt=%.3f",
                target, measured, power, dt);
    }


    private void updateShooting() {
        switch (shootState) {
            case IDLE:
                outtaking = false;
                return;

            case AIM: {
                outtaking = true;
                turret.update();

                // If turret is aimed, continue
                if (turret.isAimed()) {
                    shootState = ShootState.SET_SERVO;
                    shootTimer.reset();
                }

                telemetry.addData("turret", "aiming...");
                break;
            }

            case SET_SERVO: {
                outtaking = true;
                // Move servo to the next desired outtake position
                // int posIdx = shotOrder[shotIndex];  // SPINDEXER DISABLED
//   // SPINDEXER DISABLED
                // posIdx = (posIdx + 1) % 3; // Spindexer Outtake is offset by 1.  // SPINDEXER DISABLED
                // posIdx = Math.max(0, Math.min(posIdx, spindexerPosOuttake.length - 1));  // SPINDEXER DISABLED
                // spindexer.setPosition(spindexerPosOuttake[posIdx]);  // SPINDEXER DISABLED

                // Single-shot: always use the "first shot" RPM
                setLauncherRPM(TARGET_RPM_FIRST);

                rpmStableTimer.reset();
                shootTimer.reset();
                shootState = ShootState.SPINUP;
                break;
            }

            case SPINUP: {
                double targetRpm = TARGET_RPM_FIRST;
                long needed = FIRST_SPINUP_MS;

                boolean atSpeed = launcherAtSpeed(targetRpm);

                if (!atSpeed) {
                    rpmStableTimer.reset(); // must be continuously stable
                }

                boolean stableEnough = atSpeed && rpmStableTimer.milliseconds() >= STABLE_MS;
                boolean timedOut = shootTimer.milliseconds() >= needed;

                if (stableEnough || timedOut) {
                    // Feed one ball into the launcher
                    bottomFlywheel.setPower(vcPower(FEED_POWER));
                    topFlywheel.setPower(vcPower(FEED_POWER));

                    shootTimer.reset();
                    shootState = ShootState.FIRE;
                }

                telemetry.addData("launcherRPM", "%.0f / %.0f", getLauncherRPM(), targetRpm);
                telemetry.addData("stableMs", "%.0f", rpmStableTimer.milliseconds());
                break;
            }


            case FIRE: {
                if (shootTimer.milliseconds() >= FEED_MS) {
                    bottomFlywheel.setPower(0);
                    topFlywheel.setPower(0);

                    shootTimer.reset();
                    shootState = ShootState.RECOVER;
                }
                break;
            }


            case RECOVER: {
                double targetRpm = TARGET_RPM_FIRST;

                // Prefer RPM recovery; also keep a minimum delay
                boolean recovered = launcherAtSpeed(targetRpm);
                if ((shootTimer.milliseconds() >= RECOVER_MS) || recovered) {
                    // Single-shot: end the cycle
                    shootState = ShootState.IDLE;
                    shotIndex = 0;
                    stopShooter();
                    outtaking = false;
                }
                break;
            }

        }
    }

    private void stopShooter() {
        launcherControlEnabled = false;
        launcherTargetTicksPerSec = 0.0;
        launcher.setPower(0.0);
        bottomFlywheel.setPower(0.0);
        topFlywheel.setPower(0.0);
    }

}

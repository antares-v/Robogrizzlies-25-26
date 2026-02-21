package org.firstinspires.ftc.teamcode.mech;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.mech.Auto.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.mech.CV.ColorDetection;
import org.firstinspires.ftc.teamcode.mech.movement.movement;
import org.firstinspires.ftc.teamcode.mech.control.CustomPIDF;
import org.firstinspires.ftc.teamcode.mech.control.TurretController;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Collections;


@TeleOp
public class MainTeleop extends LinearOpMode {

    // Hardware maps
    private movement drive;
    private CRServo bottomFlywheel, topFlywheel;
    // private Servo spindexer;  // SPINDEXER DISABLED
    private DcMotorEx backIntake, frontIntake, launcher;

    // Spindexer positions
    // private final double[] spindexerPosIntake  = {0.00, 0.38, 0.79};  // SPINDEXER DISABLED
    // private final double[] spindexerPosOuttake = {0.19, 0.59, 0.99};  // SPINDEXER DISABLED

    // Ball tracking
    private final List<String> ballcols = new ArrayList<>();
    private final ColorDetection colorSensor = new ColorDetection();
    private int i = 0;

    private double Kp;
    private final ElapsedTime spintime = new ElapsedTime();

    // Button tracking
    private boolean dLeftPrev = false, dRightPrev = false;
    private boolean dUpPrev = false, dDownPrev = false;
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
    private enum ShootState { IDLE, START, SPINUP, FIRE, RECOVER }
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
    private static final double TARGET_RPM = 1500.0;

    // Battery + launcher velocity compensation
    private static final double NOMINAL_VOLTAGE = 12.0;
    private PIDFCoefficients baseLauncherPIDF;
    private double launcherTicksPerRev = 28;

    // "At speed" logic
    private final ElapsedTime rpmStableTimer = new ElapsedTime();
    private static final long STABLE_MS = 100;           // must be at speed this long before feeding
    private static final double RPM_TOL_FRAC = 0.05;     // +/-3% window around target

    // Feeder behavior (CRServos that push ball into launcher)
    private static final double FEED_POWER = 1.0;       // tune (0.6–1.0)
    private static final long FEED_MS = 7500;

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

    private CRServo turretYaw;
    private Servo turretPitch;
    private AnalogInput turretYawEnc;
    private TurretController turret;

    private PinpointLocalizer localizer;
    private Pose2d robotPos;

    // tune values
    private static double LAUNCH_kP = 0.005;
    private static double LAUNCH_kI = 0.00005;
    private static double LAUNCH_kD = 0.00003;

    // kF will be computed from motor max speed at init, but you can override if you want:
    private static double LAUNCH_kF = -1.0; // -1 = auto compute

    // Limelight AprilTag detection
    private Limelight3A limelight;

    // TODO: Measure these offsets
    private static final double LL_X_IN = 0.0;
    private static final double LL_Y_IN = 8.0;
    // Limelight yaw relative to robot forward (radians). Forward-facing = 0.
    private static final double LL_YAW_RAD = 0.0;
    // Limelight tx is typically +right. Robot frame here uses +left.
    private static final double TX_TO_ROBOT_LEFT_SIGN = -1.0;
    // Used only when pose range is unavailable for a detected tag.
    private static final double DEFAULT_TAG_RANGE_IN = 48.0;
    // Persistent offset from robot-forward frame to turret frame.
    private static final double TURRET_YAW_FORWARD_OFFSET_DEG = 15;
    // Start slightly lower so compensation is stronger (can be tuned live).
    private static final double TURRET_YAW_ENC_DEG_PER_REV = 110.0;

    // Last seen tag position in field coordinates (inches)
    private boolean hasLastTagField = false;
    private double lastTagFieldX = 0.0;
    private double lastTagFieldY = 0.0;

    // Helpers
    private static double deadzone(double v, double dz) {
        return (Math.abs(v) < dz) ? 0 : v;
    }


    private static double angleWrapRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double angleWrapDeg(double a) {
        while (a > 180.0) a -= 360.0;
        while (a <= -180.0) a += 360.0;
        return a;
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
        turretYaw  = hardwareMap.get(CRServo.class, "turretYaw");
        turretPitch = hardwareMap.get(Servo.class, "turretPitch");
        turretYawEnc = hardwareMap.get(AnalogInput.class, "turretYawEnc");;


        launcher.setDirection(DcMotorEx.Direction.REVERSE);

        localizer = new PinpointLocalizer(hardwareMap, 0.00199746322, new Pose2d(0, 0, Math.toRadians(90)));

        turret = new TurretController(turretYaw, turretPitch, turretYawEnc);
        turret.yawEncoderDegPerRev = TURRET_YAW_ENC_DEG_PER_REV;
        turret.resetYawEstimate();
        turret.useTxForYaw = true;
        turret.txSign = 1.0;
        turret.yawRobotForwardOffsetDeg = angleWrapDeg(TURRET_YAW_FORWARD_OFFSET_DEG);

        // turret.setTargetRobotRelative(36, 10, 0);

        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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

        // Limelight init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0); // pipeline index

        waitForStart();

        baseLauncherPIDF = launcher.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Max ticks/sec = maxRPM * ticksPerRev / 60
        double maxRpm = launcher.getMotorType().getMaxRPM();
        double maxTicksPerSec = (maxRpm * launcherTicksPerRev) / 60.0;
//keep all other constans zero while testing Kp but talk to gavin about kf intergration into thes system
        double kF = 0.5;//(LAUNCH_kF > 0) ? LAUNCH_kF : (1.0 / maxTicksPerSec);

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
            boolean dUp = gamepad1.dpad_up;
            boolean dDown = gamepad1.dpad_down;

            boolean dLeftPressed  = dLeft  && !dLeftPrev;
            boolean dRightPressed = dRight && !dRightPrev;
            boolean dUpPressed = dUp && !dUpPrev;
            boolean dDownPressed = dDown && !dDownPrev;

            dLeftPrev = dLeft;
            dRightPrev = dRight;
            dUpPrev = dUp;
            dDownPrev = dDown;

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

            // 6.5) Live turret tuning
            // D-pad up/down: encoder deg/rev (tracking strength)
            // D-pad left/right: yaw frame offset
            if (turret != null) {
                if (dUpPressed) turret.yawEncoderDegPerRev = Range.clip(turret.yawEncoderDegPerRev + 1.0, 40.0, 400.0);
                if (dDownPressed) turret.yawEncoderDegPerRev = Range.clip(turret.yawEncoderDegPerRev - 1.0, 40.0, 400.0);
                if (dRightPressed) turret.yawRobotForwardOffsetDeg = angleWrapDeg(turret.yawRobotForwardOffsetDeg + 1.0);
                if (dLeftPressed) turret.yawRobotForwardOffsetDeg = angleWrapDeg(turret.yawRobotForwardOffsetDeg - 1.0);
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

            // 11) Update turret (Limelight AprilTags)
            robotPos = localizer.getPose();

            boolean tagSeen = false;
            double yawErrDeg = 0.0; // still used for telemetry
            double distIn = 0.0;
            double tagRobotXIn = 0.0;
            double tagRobotYIn = 0.0;

            LLResult result = (limelight != null) ? limelight.getLatestResult() : null;
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        telemetry.addData("detection", f.getFiducialId());
                        int id = f.getFiducialId();
                        if (id == 20 || id == 21 || id == 24) {
                            yawErrDeg = f.getTargetXDegrees();
                            boolean havePoseRange = false;
                            Pose3D tagPoseRobot = f.getTargetPoseRobotSpace();
                            if (tagPoseRobot != null) {
                                double xM = tagPoseRobot.getPosition().x;
                                double yM = tagPoseRobot.getPosition().y;
                                double zM = tagPoseRobot.getPosition().z;
                                double distM = Math.sqrt(xM*xM + yM*yM + zM*zM);
                                distIn = distM * 39.3701;
                                havePoseRange = distIn > 1.0;
                                telemetry.addData("apriltagX", xM);
                                telemetry.addData("apriltagY", yM);
                                telemetry.addData("apriltagZ", zM);
                                telemetry.addData("yawErr", yawErrDeg);
                            }

                            if (!havePoseRange) {
                                distIn = DEFAULT_TAG_RANGE_IN;
                            }

                            // Build robot-relative target using tx bearing and range.
                            double bearingCamRad = Math.toRadians(TX_TO_ROBOT_LEFT_SIGN * yawErrDeg);
                            double tagCamX = distIn * Math.cos(bearingCamRad); // forward from camera
                            double tagCamY = distIn * Math.sin(bearingCamRad); // left from camera

                            double c = Math.cos(LL_YAW_RAD);
                            double s = Math.sin(LL_YAW_RAD);
                            tagRobotXIn = LL_X_IN + (tagCamX * c - tagCamY * s);
                            tagRobotYIn = LL_Y_IN + (tagCamX * s + tagCamY * c);

                            // Save absolute field location for continued tracking after tag loss.
                            // Even if distance is fallback-estimated, this keeps "perma tracking"
                            // behavior alive after first sighting.
                            double rh = robotPos.heading.toDouble();
                            double ch = Math.cos(rh);
                            double sh = Math.sin(rh);
                            lastTagFieldX = robotPos.position.x + (tagRobotXIn * ch - tagRobotYIn * sh);
                            lastTagFieldY = robotPos.position.y + (tagRobotXIn * sh + tagRobotYIn * ch);
                            hasLastTagField = true;

                            tagSeen = true;
                            break;
                        }
                    }
                }
            }

            if (turret != null) {
                boolean memoryTrackingActive = false;
                if (tagSeen) {
                    turret.updateVisionMeasurement(tagRobotXIn, tagRobotYIn, 0.0, yawErrDeg, true);
                } else if (hasLastTagField) {
                    // Field -> robot transform (x forward, y left).
                    double dx = lastTagFieldX - robotPos.position.x;
                    double dy = lastTagFieldY - robotPos.position.y;
                    double rh = robotPos.heading.toDouble();
                    double ch = Math.cos(rh);
                    double sh = Math.sin(rh);
                    double targetRobotX =  dx * ch + dy * sh;
                    double targetRobotY = -dx * sh + dy * ch;
                    turret.setTargetRobotRelative(targetRobotX, targetRobotY, 0.0);
                    turret.updateVisionMeasurement(0.0, 0.0, 0.0, 0.0, false);
                    memoryTrackingActive = true;
                } else {
                    // No vision and nothing remembered
                    turret.updateVisionMeasurement(0.0, 0.0, 0.0, 0.0, false);
                }

                turret.update();
                telemetry.addData("trackMode", tagSeen ? "VISION" : (memoryTrackingActive ? "MEMORY" : "NONE"));
            }

            telemetry.addData("tagMemory", hasLastTagField ? "YES" : "NO");
            if (hasLastTagField) {
                telemetry.addData("lastTagField", "x=%.1f y=%.1f", lastTagFieldX, lastTagFieldY);
            }
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
            telemetry.addData("rawOutValue", turret.rawOut());
            telemetry.addData("yawErrorDeg", turret.rawYawErrorDeg());
            telemetry.addData("rawPosition", turret.rawPos());
            telemetry.addData("tagSeen", tagSeen);
            telemetry.addData("tagDistIn", "%.1f", distIn);
            telemetry.addData("turretYawOffsetDeg", "%.1f", turret.yawRobotForwardOffsetDeg);
            telemetry.addData("yawEncDegPerRev", "%.4f", turret.yawEncoderDegPerRev);
            telemetry.addData("tune", "up/down=encDegPerRev left/right=offset");
            telemetry.addData("max voltage for encoder", turret.maxVoltage());
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
        shootState = ShootState.START;
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
    private void findKp(){
        if (!launcherControlEnabled) return;

        double dt = launcherLoopTimer.seconds();
        launcherLoopTimer.reset();

        double measured = launcher.getVelocity(); // ticks/sec
        double target = launcherTargetTicksPerSec;

        double power = launcherPIDF.ZiegerZichloas(target, measured, dt);
        double period = 0;

        telemetry.addData("Error_Oscillation", "CustomPIDF.oscillation=%.3f",launcherPIDF.oscillationratio);
        telemetry.addData("Ku",Kp);
        if(launcherPIDF.oscillationratio<0.01){
            if(launcherPIDF.errorlist.size()>1000){
                int j = launcherPIDF.errorlist.indexOf(Collections.max(launcherPIDF.errorlist));
                int k = 0;
                for(int i=0;i<launcherPIDF.errorlist.size();i++){
                    if((launcherPIDF.errorlist.get(j)+.01)>launcherPIDF.errorlist.get(i) && (launcherPIDF.errorlist.get(j)-.01)<launcherPIDF.errorlist.get(i)){
                        k = i;
                    }
                }
                for(int i=k;i<launcherPIDF.errorlist.size();i++){
                    if((launcherPIDF.errorlist.get(k)+.01)>launcherPIDF.errorlist.get(i) && (launcherPIDF.errorlist.get(k)-.01)<launcherPIDF.errorlist.get(i)){
                        double frequency = 1/Math.abs((launcherPIDF.timelist.get(i)-launcherPIDF.timelist.get(k)));
                        period = 2*3.14/frequency;
                        break;
                    }
                }
                telemetry.addData("Error_Oscillation", "CustomPIDF.oscillation=%.3f",launcherPIDF.oscillationratio);
                telemetry.addData("Ku", Kp);
                telemetry.addData("Pu", "Period=%.3f",period);
                telemetry.addLine("Ku found, testing over");
                stopShooter();
            }
        } else{
            if(launcherPIDF.errorlist.size()>10){
                Kp+=0.001;
                launcherPIDF = new CustomPIDF(Kp, LAUNCH_kI, LAUNCH_kD, launcherPIDF.kF);
            }
        }
        // Optional: voltage compensation (helps keep behavior consistent)
        double scale = NOMINAL_VOLTAGE / batteryVoltage();
        power = Range.clip(power, -1.0, 1.0);

        launcher.setPower(power);

        telemetry.addData("L PID", "t=%.0f m=%.0f pwr=%.2f dt=%.3f",
                target, measured, power, dt);
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

            case START: {
                outtaking = true;
                // Move servo to the next desired outtake position
                // int posIdx = shotOrder[shotIndex];  // SPINDEXER DISABLED
//   // SPINDEXER DISABLED
                // posIdx = (posIdx + 1) % 3; // Spindexer Outtake is offset by 1.  // SPINDEXER DISABLED
                // posIdx = Math.max(0, Math.min(posIdx, spindexerPosOuttake.length - 1));  // SPINDEXER DISABLED
                // spindexer.setPosition(spindexerPosOuttake[posIdx]);  // SPINDEXER DISABLED

                // Single-shot: always use the "first shot" RPM
                setLauncherRPM(TARGET_RPM);

                rpmStableTimer.reset();
                shootTimer.reset();
                shootState = ShootState.SPINUP;
                break;
            }

            case SPINUP: {
                double targetRpm = TARGET_RPM;
                long needed = FIRST_SPINUP_MS;

                boolean atSpeed = launcherAtSpeed(targetRpm);

                if (!atSpeed) {
                    rpmStableTimer.reset(); // must be continuously stable
                }

                boolean stableEnough = atSpeed && rpmStableTimer.milliseconds() >= STABLE_MS;
                boolean timedOut = shootTimer.milliseconds() >= needed;

                if (stableEnough || timedOut) {
                    // Feed one ball into the launcher
                    bottomFlywheel.setPower(FEED_POWER);
                    topFlywheel.setPower(FEED_POWER);

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
                double targetRpm = TARGET_RPM;

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

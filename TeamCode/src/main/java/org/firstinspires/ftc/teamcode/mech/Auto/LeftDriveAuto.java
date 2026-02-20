package org.firstinspires.ftc.teamcode.mech.Auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mech.control.CustomPIDF;
import org.firstinspires.ftc.teamcode.mech.control.TurretController;

import java.util.List;

@Autonomous(name = "Left Drive Auto")
public class LeftDriveAuto extends LinearOpMode {
    private static final class Config {
        // Spindexer positions (disabled; robot has no spindexer)
        // static final double[] SPINDEX_INTAKE  = {0.00, 0.38, 0.79};
        // static final double[] SPINDEX_OUTTAKE = {0.19, 0.59, 0.99};

        static final double LAUNCHER_TICKS_PER_REV = 28.0;
        // target RPMs (tune these)
        static final double TARGET_RPM_FIRST = 1900.0; // example, tune to match desired shot power
        static final double TARGET_RPM_NEXT  = 2100.0; // often same as first, tune as needed

        // computed velocity targets (ticks per second)
        static final double TARGET_VEL_FIRST = TARGET_RPM_FIRST * LAUNCHER_TICKS_PER_REV / 60.0;
        static final double TARGET_VEL_NEXT  = TARGET_RPM_NEXT  * LAUNCHER_TICKS_PER_REV / 60.0;

        // when this fraction of target is reached we consider it spun up
        static final double VEL_THRESHOLD_FRAC = 0.95;

        // Ball layout
        static final double BALL_SPACING = 6.0;
        static final double FIRST_BALL_X = -43;
        static final double ROBOT_OFFSET = 8.0;

        // Row Ys
        static final double FAR_ROW_Y   = 5;
        static final double MID_ROW_Y   = -19;
        static final double CLOSE_ROW_Y = -43;

        // Motion constraint
        static final double MOTION_VEL = 70.0;
        static final double COLLECT_VEL = 30.0;

        // Poses
        static final Pose2d START_POSE  = new Pose2d(-12, -60, Math.toRadians(90));
        static final Pose2d SHOOT_POSE  = new Pose2d(-12, -25, Math.toRadians(110));
        static final double COLLECT_HEADING_RAD = Math.toRadians(180);

        // Shooter timing
        static final double LAUNCHER_SPINUP_SEC = 1.0;
        static final double FIRE_WINDOW_SEC     = 0.5;
        static final double LAUNCH_kP = 0.00025;
        static final double LAUNCH_kI = 0.0000008;
        static final double LAUNCH_kD = 0.00001;
        static final double LAUNCH_kF = -1.0; // -1 => auto-compute from motor max speed

        // Row X offsets
        static final float[] ROW_X_MULTS = { +2.0f, -0.5f, -1.5f, -2.5f };
    }

    private static final class RowSpec {
        final String name;
        final double y;
        final boolean[] waitBeforeSegment;

        RowSpec(String name, double y, boolean s1, boolean s2, boolean s3) {
            this.name = name;
            this.y = y;
            this.waitBeforeSegment = new boolean[] { s1, s2, s3 };
        }
    }
    private static final class RobotHW {
        final CRServo bottomFlywheel, topFlywheel;
        // final Servo spindexer; // disabled

        // runtime fields
        static double currentTargetVel = 0.0;
        final DcMotor backIntake, frontIntake;
        final DcMotorEx launcher;
        final Servo turretYaw;
        final Servo turretPitch;
        final TurretController turret;
        final Limelight3A limelight;
        final CustomPIDF launcherPIDF;
        final ElapsedTime launcherLoopTimer = new ElapsedTime();
        final double launcherTicksPerRev;
        double launcherTargetTicksPerSec = 0.0;
        boolean launcherControlEnabled = false;

        RobotHW(LinearOpMode opMode) {
            backIntake = opMode.hardwareMap.get(DcMotor.class, "backIntake");
            frontIntake = opMode.hardwareMap.get(DcMotor.class, "frontIntake");
            launcher = opMode.hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            bottomFlywheel = opMode.hardwareMap.get(CRServo.class, "bottomFlywheel");
            topFlywheel = opMode.hardwareMap.get(CRServo.class, "topFlywheel");
            // spindexer = opMode.hardwareMap.get(Servo.class, "spindexer");

            turretYaw = opMode.hardwareMap.get(Servo.class, "turretYaw");
            turretPitch = opMode.hardwareMap.get(Servo.class, "turretPitch");
            turret = new TurretController(turretYaw, turretPitch);
            limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);

            double ticksPerRev = launcher.getMotorType().getTicksPerRev();
            launcherTicksPerRev = (ticksPerRev > 0.0) ? ticksPerRev : Config.LAUNCHER_TICKS_PER_REV;
            double maxRpm = launcher.getMotorType().getMaxRPM();
            double maxTicksPerSec = Math.max(1.0, (maxRpm * launcherTicksPerRev) / 60.0);
            double kF = (Config.LAUNCH_kF > 0.0) ? Config.LAUNCH_kF : (1.0 / maxTicksPerSec);
            launcherPIDF = new CustomPIDF(Config.LAUNCH_kP, Config.LAUNCH_kI, Config.LAUNCH_kD, kF);
            launcherPIDF.iMax = 0.35;
            launcherPIDF.reset();
            launcherLoopTimer.reset();
        }

        void setIntakePower(double pwr) {
            backIntake.setPower(pwr);
            frontIntake.setPower(pwr);
        }

        void stopFlywheels() {
            // Keep directions consistent; power=0 is what matters
            bottomFlywheel.setPower(0);
            topFlywheel.setPower(0);
        }

        void startFlywheelsForShooting() {
            // Same intent as your code: left forward, right reverse
            bottomFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            topFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomFlywheel.setPower(1);
            topFlywheel.setPower(1);
        }

        void setLauncherRPM(double rpm) {
            launcherTargetTicksPerSec = rpm * launcherTicksPerRev / 60.0;
            launcherControlEnabled = (rpm > 0.0);
            launcherPIDF.reset();
            launcherLoopTimer.reset();
        }

        void updateLauncherPIDF() {
            if (!launcherControlEnabled) return;
            double dt = launcherLoopTimer.seconds();
            launcherLoopTimer.reset();
            if (dt <= 1e-6) dt = 1e-3;
            double measured = launcher.getVelocity();
            double power = launcherPIDF.update(launcherTargetTicksPerSec, measured, dt);
            launcher.setPower(power);
        }

        void stopLauncherControl() {
            launcherControlEnabled = false;
            launcherTargetTicksPerSec = 0.0;
            launcher.setPower(0.0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHW hw = new RobotHW(this);

        TranslationalVelConstraint motionVel = new TranslationalVelConstraint(Config.MOTION_VEL);
        TranslationalVelConstraint collectVel = new TranslationalVelConstraint(Config.COLLECT_VEL);

        // Define row behaviors
        RowSpec FAR   = new RowSpec("FAR",   Config.FAR_ROW_Y,   false, false, true);
        RowSpec MID   = new RowSpec("MID",   Config.MID_ROW_Y,   true,  true,  true);
        RowSpec CLOSE = new RowSpec("CLOSE", Config.CLOSE_ROW_Y, true,  true,  true);

        // RoadRunner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, Config.START_POSE);

        // Build paths
        Vector2d[] farPts   = makeRowPoints(FAR.y);
        Vector2d[] midPts   = makeRowPoints(MID.y);
        Vector2d[] closePts = makeRowPoints(CLOSE.y);

        Pose2d[] farEnds   = makeRowEndPoses(farPts);
        Pose2d[] midEnds   = makeRowEndPoses(midPts);
        Pose2d[] closeEnds = makeRowEndPoses(closePts);

        // start => shoot
        Action toShootInitially = drive.actionBuilder(Config.START_POSE)
                .splineToLinearHeading(Config.SHOOT_POSE, Math.toRadians(135), motionVel)
                .build();
        Action goBackToWhereYouCameFrom = drive.actionBuilder(Config.SHOOT_POSE)
                .splineToLinearHeading(Config.START_POSE, Math.toRadians(0), motionVel)
                .build();
        // shoot => start of each row
        Action toFarRowStart = drive.actionBuilder(Config.SHOOT_POSE)
                .strafeToLinearHeading(farPts[0], Config.COLLECT_HEADING_RAD, motionVel)
                .build();

        Action toMidRowStart = drive.actionBuilder(Config.SHOOT_POSE)
                .strafeToLinearHeading(midPts[0], Config.COLLECT_HEADING_RAD, motionVel)
                .build();

        Action toCloseRowStart = drive.actionBuilder(Config.SHOOT_POSE)
                .strafeToLinearHeading(closePts[0], Config.COLLECT_HEADING_RAD, motionVel)
                .build();

        // Collect across each row
        Action[] farCollect = buildCollectAcrossRow(
                drive, hw, farPts, farEnds, FAR.waitBeforeSegment, collectVel
        );
        Action[] midCollect = buildCollectAcrossRow(
                drive, hw, midPts, midEnds, MID.waitBeforeSegment, collectVel
        );
        Action[] closeCollect = buildCollectAcrossRow(
                drive, hw, closePts, closeEnds, CLOSE.waitBeforeSegment, collectVel
        );

        // From end of each row -> shoot
        Action farEndToShoot = drive.actionBuilder(farEnds[3])
                .splineToLinearHeading(Config.SHOOT_POSE, Math.toRadians(135), motionVel)
                .build();

        Action midEndToShoot = drive.actionBuilder(midEnds[3])
                .splineToLinearHeading(Config.SHOOT_POSE, Math.toRadians(135), motionVel)
                .build();

        Action closeEndToShoot = drive.actionBuilder(closeEnds[3])
                .splineToLinearHeading(Config.SHOOT_POSE, Math.toRadians(135), motionVel)
                .build();

        // auto chain
        Action autonomousChain = new SequentialAction(
                toShootInitially,
                goBackToWhereYouCameFrom
        );

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(autonomousChain);
    }
    private static Vector2d[] makeRowPoints(double rowY) {
        Vector2d[] pts = new Vector2d[4];
        for (int i = 0; i < 4; i++) {
            double x = Config.FIRST_BALL_X + Config.ROW_X_MULTS[i] * Config.BALL_SPACING + Config.ROBOT_OFFSET;
            pts[i] = new Vector2d(x, rowY);
        }
        return pts;
    }

    private static Pose2d[] makeRowEndPoses(Vector2d[] pts) {
        Pose2d[] ends = new Pose2d[4];
        for (int i = 0; i < 4; i++) {
            ends[i] = new Pose2d(pts[i].x, pts[i].y, Config.COLLECT_HEADING_RAD);
        }
        return ends;
    }
    private Action[] buildCollectAcrossRow(
            MecanumDrive drive,
            RobotHW hw,
            Vector2d[] rowPts,
            Pose2d[] rowEnds,
            boolean[] waitBeforeSegment,
            TranslationalVelConstraint vel
    ) {
        Action[] segments = new Action[3];
        // Keep driving continuously across the entire row while intake stays on.
        Action driveAll = drive.actionBuilder(rowEnds[0])
                .strafeToLinearHeading(rowPts[1], Config.COLLECT_HEADING_RAD, vel)
                .strafeToLinearHeading(rowPts[2], Config.COLLECT_HEADING_RAD, vel)
                .strafeToLinearHeading(rowPts[3], Config.COLLECT_HEADING_RAD, vel)
                .build();
        segments[0] = runIntakeWhileAction(hw, driveAll);
        segments[1] = noOpAction();
        segments[2] = noOpAction();
        return segments;
    }

    private static Action runIntakeWhileAction(RobotHW hw, Action driveAction) {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    hw.setIntakePower(1);
                }
                if (driveAction.run(packet)) {
                    return true;
                }
                hw.setIntakePower(0);
                return false;
            }
        };
    }

    private static Action noOpAction() {
        return packet -> false;
    }
    private enum Phase { START_BALL, AIM, SPINUP, FIRE, ADVANCE, DONE }

    private static Action shootThreeBalls(RobotHW hw) {
        return new Action() {
            private Phase phase = Phase.START_BALL;
            private int ballIndex = 0;

            private ElapsedTime phaseTimer = null;

            private void resetTimer() {
                phaseTimer = new ElapsedTime();
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                if (phaseTimer == null) resetTimer();
                hw.updateLauncherPIDF();

                switch (phase) {

                    case START_BALL: {
                        // Start launcher and set initial spindex position for this ball
                        hw.setLauncherRPM((ballIndex == 0) ? Config.TARGET_RPM_FIRST : Config.TARGET_RPM_NEXT);
                        // hw.spindexer.setPosition(Config.SPINDEX_OUTTAKE[ballIndex]);
                        hw.stopFlywheels();

                        phase = Phase.AIM;;
                        resetTimer();
                        return true;
                    }
                case AIM: {
                    boolean tagSeen = false;
                    double yawErrDeg = 0.0;
                    double xIn = 0.0;
                    double yIn = 0.0;
                    double zIn = 0.0;

                    LLResult result = (hw.limelight != null) ? hw.limelight.getLatestResult() : null;
                    if (result != null && result.isValid()) {
                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        if (fiducials != null) {
                            for (LLResultTypes.FiducialResult f : fiducials) {
                                int id = f.getFiducialId();
                                if (id == 20 || id == 21 || id == 24) {
                                    yawErrDeg = f.getTargetXDegrees();
                                    Pose3D tagPoseRobot = f.getTargetPoseRobotSpace();
                                    if (tagPoseRobot != null) {
                                        double xM = tagPoseRobot.getPosition().x;
                                        double yM = tagPoseRobot.getPosition().y;
                                        double zM = tagPoseRobot.getPosition().z;
                                        xIn = xM * 39.3701;
                                        yIn = yM * 39.3701;
                                        zIn = zM * 39.3701;
                                    }
                                    tagSeen = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (hw.turret != null) {
                        hw.turret.updateVisionMeasurement(xIn, yIn, zIn, yawErrDeg, tagSeen);
                        hw.turret.update();
                    }
                    if (hw.turret.isAimed() || phaseTimer.seconds() > 1.5) {
                        phase = Phase.SPINUP;
                        resetTimer();
                    }
                    return true;
                }


                    case SPINUP: {
                        double vT = (ballIndex == 0) ? Config.TARGET_VEL_FIRST : Config.TARGET_VEL_NEXT;
                        if (hw.launcher.getVelocity() < vT * Config.VEL_THRESHOLD_FRAC && phaseTimer.seconds() < Config.LAUNCHER_SPINUP_SEC) {
                           return true;}
                        phase = Phase.FIRE;
                        resetTimer();
                        return true;
                    }

                    case FIRE: {
                        // Run flywheels during the fire window
                        hw.startFlywheelsForShooting();

                        if (phaseTimer.seconds() < Config.FIRE_WINDOW_SEC) return true;

                        phase = Phase.ADVANCE;
                        resetTimer();
                        return true;
                    }

                    case ADVANCE: {
                        hw.stopFlywheels();

                        if (ballIndex < 2) {
                            ballIndex++;
                            phase = Phase.START_BALL;
                            resetTimer();
                            return true;
                        }

                        phase = Phase.DONE;
                        resetTimer();
                        return true;
                    }

                    case DONE: {
                        hw.stopFlywheels();
                        hw.stopLauncherControl();
                        // hw.spindexer.setPosition(Config.SPINDEX_OUTTAKE[0]);
                        return false;
                    }
                }

                return false;
            }
        };
    }
}

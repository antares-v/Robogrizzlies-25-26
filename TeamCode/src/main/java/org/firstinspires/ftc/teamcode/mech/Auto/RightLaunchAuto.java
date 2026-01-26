package org.firstinspires.ftc.teamcode.mech.Auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Right Launch Auto")
public class RightLaunchAuto extends LinearOpMode {
    private static final class Config {
        // Spindexer positions
        static final double[] SPINDEX_INTAKE  = {0.00, 0.38, 0.79};
        static final double[] SPINDEX_OUTTAKE = {0.19, 0.59, 0.99};

        // Ball layout
        static final double BALL_SPACING = 6.0;
        static final double FIRST_BALL_X = 43;
        static final double ROBOT_OFFSET = 8.0;

        // Row Ys
        static final double FAR_ROW_Y   = 5;
        static final double MID_ROW_Y   = -19;
        static final double CLOSE_ROW_Y = -43;

        // Wait before some strafes
        static final double BALL_WAIT_SEC = 0.5;

        // Motion constraint
        static final double MOTION_VEL = 70.0;
        static final double COLLECT_VEL = 30.0;

        // Poses
        static final Pose2d START_POSE  = new Pose2d(48, 48, Math.toRadians(45));
        static final Pose2d SHOOT_POSE  = new Pose2d(37, 37, Math.toRadians(45));
        static final double COLLECT_HEADING_RAD = Math.toRadians(0);

        // Intake timing
        static final double INTAKE_RUN_SEC = 3.0;

        // Shooter timing
        static final double LAUNCHER_FIRST_SPINUP_SEC = 1.5;
        static final double LAUNCHER_SPINUP_SEC = 0.7;
        static final double FIRE_WINDOW_SEC     = 0.7;
        static final double LAUNCHER_TICKS_PER_REV = 28.0;
        // target RPMs (tune these)
        static final double TARGET_RPM_FIRST = 550.0; // example, tune to match desired shot power
        static final double TARGET_RPM_NEXT  = 420.0; // often same as first, tune as needed

        // computed velocity targets (ticks per second)
        static final double TARGET_VEL_FIRST = TARGET_RPM_FIRST * LAUNCHER_TICKS_PER_REV / 60.0;
        static final double TARGET_VEL_NEXT  = TARGET_RPM_NEXT  * LAUNCHER_TICKS_PER_REV / 60.0;

        // when this fraction of target is reached we consider it spun up
        static final double VEL_THRESHOLD_FRAC = 0.95;

        // runtime fields
        static double currentTargetVel = 0.0;

        // Row X offsets
        static final float[] ROW_X_MULTS = { -3.5f, +0.5f, +1.5f, +2.5f };
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
        final CRServo leftFlywheel, rightFlywheel;
        final Servo spindexer;
        final DcMotor leftIntake, rightIntake;
        final DcMotorEx launcher;

        RobotHW(LinearOpMode opMode) {
            leftIntake = opMode.hardwareMap.get(DcMotor.class, "leftIntake");
            rightIntake = opMode.hardwareMap.get(DcMotor.class, "rightIntake");
            launcher = opMode.hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

            leftFlywheel = opMode.hardwareMap.get(CRServo.class, "leftFlywheel");
            rightFlywheel = opMode.hardwareMap.get(CRServo.class, "rightFlywheel");
            spindexer = opMode.hardwareMap.get(Servo.class, "spindexer");
        }

        void setIntakePower(double pwr) {
            leftIntake.setPower(pwr);
            rightIntake.setPower(pwr);
        }

        void stopFlywheels() {
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
        }

        void startFlywheelsForShooting() {
            // Same intent as your code: left forward, right reverse
            leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFlywheel.setPower(1);
            rightFlywheel.setPower(1);
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
                .splineToLinearHeading(Config.SHOOT_POSE, Math.toRadians(45), motionVel)
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
                .splineToLinearHeading(Config.SHOOT_POSE, Math.toRadians(45), motionVel)
                .build();

        Action midEndToShoot = drive.actionBuilder(midEnds[3])
                .splineToLinearHeading(Config.SHOOT_POSE, Math.toRadians(45), motionVel)
                .build();

        Action closeEndToShoot = drive.actionBuilder(closeEnds[3])
                .splineToLinearHeading(Config.SHOOT_POSE, Math.toRadians(45), motionVel)
                .build();

        // auto chain
        Action autonomousChain = new SequentialAction(
                toShootInitially,
                shootThreeBalls(hw)

                // toFarRowStart,
                /*farCollect[0], farCollect[1], farCollect[2],
                farEndToShoot,
                shootThreeBalls(hw),

                toMidRowStart,
                midCollect[0], midCollect[1], midCollect[2],
                midEndToShoot,
                shootThreeBalls(hw),

                toCloseRowStart,
                closeCollect[0], closeCollect[1], closeCollect[2],
                closeEndToShoot,
                shootThreeBalls(hw)*/
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

        for (int seg = 0; seg < 3; seg++) {
            Pose2d startPose = rowEnds[seg];
            Vector2d target = rowPts[seg + 1];

            // Build drive segment with optional wait
            com.acmerobotics.roadrunner.TrajectoryActionBuilder builder = drive.actionBuilder(startPose);
            if (waitBeforeSegment[seg]) {
                builder = builder.waitSeconds(Config.BALL_WAIT_SEC);
            }
            Action driveSeg = builder
                    .strafeToLinearHeading(target, Config.COLLECT_HEADING_RAD, vel)
                    .build();

            // Build intake segment
            Action intakeSeg = intakeForSlot(hw, seg);

            segments[seg] = new ParallelAction(driveSeg, intakeSeg);
        }

        return segments;
    }
    private static Action intakeForSlot(RobotHW hw, int slotIndex) {
        return new Action() {
            private boolean initialized = false;
            private ElapsedTime timer;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    hw.spindexer.setPosition(Config.SPINDEX_INTAKE[slotIndex]);
                    hw.setIntakePower(1);
                    timer = new ElapsedTime();
                    initialized = true;
                }

                if (timer.seconds() < Config.INTAKE_RUN_SEC) {
                    return true;
                }

                hw.setIntakePower(0);
                return false;
            }
        };
    }
    private enum Phase { START_BALL, SPINUP, FIRE, ADVANCE, DONE }

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

                switch (phase) {

                    case START_BALL: {
                        // Start launcher and set initial spindex position for this ball
                        hw.stopFlywheels();
                        hw.launcher.setVelocity((ballIndex == 0) ? Config.TARGET_VEL_FIRST : Config.TARGET_VEL_NEXT);
                        hw.spindexer.setPosition(Config.SPINDEX_OUTTAKE[ballIndex]);

                        phase = Phase.SPINUP;
                        resetTimer();
                        return true;
                    }
                    case SPINUP: {
                        double vT = (ballIndex == 0) ? Config.TARGET_VEL_FIRST : Config.TARGET_VEL_NEXT;
                        if (ballIndex == 0) {
                            if ((hw.launcher.getVelocity() < vT * Config.VEL_THRESHOLD_FRAC && phaseTimer.seconds() < Config.LAUNCHER_FIRST_SPINUP_SEC)){
                                return true;}
                        }
                        else {
                            if ((hw.launcher.getVelocity() < vT * Config.VEL_THRESHOLD_FRAC && phaseTimer.seconds() < Config.LAUNCHER_SPINUP_SEC)){
                                return true;}
                        }
                        phase = Phase.FIRE;
                        resetTimer();
                        return true;
                    }
                    case FIRE: {
                        // Run flywheels during the fire window
                        hw.startFlywheelsForShooting();

                        if (phaseTimer.seconds() < Config.FIRE_WINDOW_SEC) return true;
                        hw.spindexer.setPosition(Config.SPINDEX_OUTTAKE[ballIndex]);
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
                        hw.launcher.setPower(0);
                        hw.spindexer.setPosition(Config.SPINDEX_OUTTAKE[0]);
                        return false;
                    }
                }

                return false;
            }
        };
    }
}

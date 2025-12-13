package org.firstinspires.ftc.teamcode.mech.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Left Drive Auto")
//ts is the auto for one side you can prolly mirror for the other side
public class LeftDriveAuto extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo leftFlywheel, rightFlywheel;
        Servo spindexer;
        DcMotor leftIntake, rightIntake, launcher;

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel = hardwareMap.get(CRServo.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(CRServo.class, "rightFlywheel");
        spindexer = hardwareMap.get(Servo.class, "spindexer");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        double[] spindexerPosIntake = {0,0.38,0.79};
        double[] spindexerPosOuttake = {0.19,0.59,0.99};
        double interballDistance = 6;
        double farRowY = 12;
        double midRowY = -12;
        double closeRowY = -36;
        double firstBallX = -38;
        double ballWaiting = 0.5;
        TranslationalVelConstraint tvConstraint = new TranslationalVelConstraint(25.0);

        class RobotMechanisms {
            // 2. Define the method that returns an Action

            Pose2d Startingpos = new Pose2d(-12, -60, Math.toRadians(135));
            //Pose2d Startingpos = new Pose2d(24,-60, Math.toRadians(45));
            Pose2d shootLocation = new Pose2d(-40,40, Math.toRadians(135));
            Vector2d shootVector = new Vector2d(-40,40);
            Vector2d farRow1 = new Vector2d(firstBallX+5*interballDistance, farRowY);
            Vector2d farRow2 = new Vector2d((firstBallX-1*interballDistance), farRowY);
            Vector2d farRow3 = new Vector2d(firstBallX-2*interballDistance, farRowY);
            Vector2d farRow4 = new Vector2d(firstBallX-3*interballDistance, farRowY);
            Pose2d farRow1End = new Pose2d(firstBallX+5*interballDistance,farRowY, Math.toRadians(180));
            Pose2d farRow2End = new Pose2d(firstBallX-1*interballDistance, farRowY, Math.toRadians(180));
            Pose2d farRow3End = new Pose2d(firstBallX-2*interballDistance, farRowY, Math.toRadians(180));
            Pose2d farRow4End = new Pose2d(firstBallX-3*interballDistance, farRowY, Math.toRadians(180));
            Vector2d midRow1 = new Vector2d(firstBallX+5*interballDistance, -midRowY);
            Vector2d midRow2 = new Vector2d(firstBallX-1*interballDistance, midRowY);
            Vector2d midRow3 = new Vector2d(firstBallX-2*interballDistance, midRowY);
            Vector2d midRow4 = new Vector2d(firstBallX-3*interballDistance, midRowY);
            Pose2d midRow1End = new Pose2d(firstBallX+5*interballDistance,midRowY, Math.toRadians(180));
            Pose2d midRow2End = new Pose2d(firstBallX-1*interballDistance, midRowY, Math.toRadians(180));
            Pose2d midRow3End = new Pose2d(firstBallX-2*interballDistance, midRowY, Math.toRadians(180));
            Pose2d midRow4End = new Pose2d(firstBallX-3*interballDistance, midRowY, Math.toRadians(180));
            Vector2d closeRow1 = new Vector2d(firstBallX+5*interballDistance, closeRowY);
            Vector2d closeRow2 = new Vector2d(firstBallX-1*interballDistance, closeRowY);
            Vector2d closeRow3 = new Vector2d(firstBallX-2*interballDistance, closeRowY);
            Vector2d closeRow4 = new Vector2d(firstBallX-3*interballDistance, closeRowY);
            Pose2d closeRow1End = new Pose2d(firstBallX+5*interballDistance,closeRowY, Math.toRadians(180));
            Pose2d closeRow2End = new Pose2d(firstBallX-1*interballDistance, closeRowY, Math.toRadians(180));
            Pose2d closeRow3End = new Pose2d(firstBallX-2*interballDistance, closeRowY, Math.toRadians(180));
            Pose2d closeRow4End = new Pose2d(firstBallX-3*interballDistance, closeRowY, Math.toRadians(180));
            MecanumDrive Drivetrain = new MecanumDrive(hardwareMap, Startingpos);
            public Action collectRowOfBalls(int spinpos) {
                return new Action() {
                    private boolean initialized = false;
                    private ElapsedTime timer;


                    @Override
                    public boolean run(TelemetryPacket packet) {
                        // 1. Start the motors on the very first run
                        if (!initialized) {
                            spindexer.setPosition(spindexerPosIntake[spinpos]);
                            rightIntake.setPower(-1);
                            leftIntake.setPower(-1);
                            timer = new ElapsedTime();
                            initialized = true;
                        }

                        // 2. Check if 2 seconds have passed
                        if (timer.seconds() < 5.0) {
                            // Return true to keep this Action running
                            return true;
                        } else {
                            // 3. Stop motors and finish the Action
                            rightIntake.setPower(0);
                            leftIntake.setPower(0);

                            return false; // Return false to signal we are done
                        }
                    }
                };
            }

            public Action Shoot() {
                return new Action() {
                    private boolean initialized = false;
                    private ElapsedTime timer;
                    private ElapsedTime timer2;
                    int i = 0;
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        if (timer == null) {
                            timer = new ElapsedTime();
                            return true;
                        }
                        if (!initialized) {
                            launcher.setPower(1);
                            initialized = true;
                            spindexer.setPosition(spindexerPosOuttake[i]);
                            return true;
                        }
                        if (timer2 == null) {
                            timer2 = new ElapsedTime();
                            return true;
                        }
                        if (timer.seconds() < 1.0){
                            return true;
                        }
                        if (timer.seconds() > 1.0 && timer2.seconds() < 3.0) {
                            leftFlywheel.setDirection(CRServo.Direction.FORWARD);
                            leftFlywheel.setPower(1);
                            rightFlywheel.setDirection(CRServo.Direction.REVERSE);
                            rightFlywheel.setPower(1);
                            return true;
                        }
                        if (i<2) {
                            i = i + 1;
                            leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                            leftFlywheel.setPower(0);
                            rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                            rightFlywheel.setPower(0);
                            spindexer.setPosition(spindexerPosOuttake[i]);
                            timer2 = null;
                            timer = null;
                            initialized = false;
                            return true;
                        }
                        else {
                            leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                            leftFlywheel.setPower(0);
                            rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                            rightFlywheel.setPower(0);
                            launcher.setPower(0);
                            spindexer.setPosition(spindexerPosOuttake[0]);
                            return false;
                        }
                    }
                };
            };

            Action trajectoryCollectRow1_1 = Drivetrain.actionBuilder(farRow1End)
                    // .waitSeconds(ballWaiting)
                    .strafeToLinearHeading(farRow2, Math.toRadians(180), tvConstraint)
                    .build();
            Action trajectoryCollectRow1_2 = Drivetrain.actionBuilder(farRow2End)
                    // .waitSeconds(ballWaiting)
                    .strafeTo(farRow3, tvConstraint)
                    .build();
            Action trajectoryCollectRow1_3 = Drivetrain.actionBuilder(farRow3End)
                    .waitSeconds(ballWaiting)
                    .strafeTo(farRow4, tvConstraint)
                    .build();
            Action trajectoryCollectRow2_1 = Drivetrain.actionBuilder(midRow1End)
                    .waitSeconds(ballWaiting)
                    .strafeTo(midRow2, tvConstraint)
                    .build();
            Action trajectoryCollectRow2_2 = Drivetrain.actionBuilder(midRow2End)
                    .waitSeconds(ballWaiting)
                    .strafeTo(midRow3, tvConstraint)
                    .build();
            Action trajectoryCollectRow2_3 = Drivetrain.actionBuilder(midRow3End)
                    .waitSeconds(ballWaiting)
                    .strafeTo(midRow4, tvConstraint)
                    .build();
            Action trajectoryCollectRow3_1 = Drivetrain.actionBuilder(closeRow1End)
                    .waitSeconds(ballWaiting)
                    .strafeTo(closeRow2, tvConstraint)
                    .build();
            Action trajectoryCollectRow3_2 = Drivetrain.actionBuilder(closeRow2End)
                    .waitSeconds(ballWaiting)
                    .strafeTo(closeRow3, tvConstraint)
                    .build();
            Action trajectoryCollectRow3_3 = Drivetrain.actionBuilder(closeRow3End)
                    .waitSeconds(ballWaiting)
                    .strafeTo(closeRow4, tvConstraint)
                    .build();

            // 2. Combine the trajectory and your intake action
            Action Pickup1_1 = new ParallelAction(
                    trajectoryCollectRow1_1,
                    collectRowOfBalls(0)
            );
            Action Pickup1_2  = new ParallelAction(
                    trajectoryCollectRow1_2,
                    collectRowOfBalls(1)
            );

            Action Pickup1_3  = new ParallelAction(
                    collectRowOfBalls(2),
                    trajectoryCollectRow1_3
            );
            Action Pickup2_1 = new ParallelAction(
                    trajectoryCollectRow2_1,
                    collectRowOfBalls(0)
            );
            Action Pickup2_2 = new ParallelAction(
                    trajectoryCollectRow2_2,
                    collectRowOfBalls(1)
            );
            Action Pickup2_3 = new ParallelAction(
                    trajectoryCollectRow2_3,
                    collectRowOfBalls(2)
            );

            Action Pickup3_1 = new ParallelAction(
                    trajectoryCollectRow3_1,
                    collectRowOfBalls(0)
            );
            Action Pickup3_2 = new ParallelAction(
                    trajectoryCollectRow3_2,
                    collectRowOfBalls(1)
            );
            Action Pickup3_3 = new ParallelAction(
                    trajectoryCollectRow3_3,
                    collectRowOfBalls(2)
            );

            Action Trajectoryrow1 = Drivetrain.actionBuilder(shootLocation)
                    .strafeTo(farRow1)
                    .build();

            Action Trajectoryrow2 = Drivetrain.actionBuilder(shootLocation)
                    .strafeTo(midRow1)
                    .build();
            Action Trajectoryrow3 = Drivetrain.actionBuilder(shootLocation)
                    .strafeTo(closeRow1)
                    .build();
            Action Shootingposa1 = Drivetrain.actionBuilder(farRow4End)
                    .splineToLinearHeading(shootLocation, Math.toRadians(135))
                    .build();
            Action Shootingposa2 = Drivetrain.actionBuilder(midRow4End)
                    .splineToLinearHeading(shootLocation, Math.toRadians(135))
                    .build();
            Action Shootingposa3 = Drivetrain.actionBuilder(closeRow4End)
                    .splineToLinearHeading(shootLocation, Math.toRadians(135))
                    .build();
            Action TrajectoryInitial = Drivetrain.actionBuilder(Startingpos)
                    .splineToLinearHeading(shootLocation, Math.toRadians(45))
                    .build();
            Action autonoumouschain = new SequentialAction(
                    TrajectoryInitial,
                    Shoot(),
                    Trajectoryrow1,
                    Pickup1_1,
                    Pickup1_2,
                    Pickup1_3,
                    Shootingposa1,
                    Shoot(),
                    Trajectoryrow2,
                    Pickup2_1,
                    Pickup2_2,
                    Pickup2_3,
                    Shootingposa2,
                    Shoot(),
                    Trajectoryrow3,
                    Pickup3_1,
                    Pickup3_2,
                    Pickup3_3,
                    Shootingposa3,
                    Shoot()
            );
        }
        waitForStart();
        RobotMechanisms Robot  = new RobotMechanisms();
        Actions.runBlocking(Robot.autonoumouschain);
    }
}
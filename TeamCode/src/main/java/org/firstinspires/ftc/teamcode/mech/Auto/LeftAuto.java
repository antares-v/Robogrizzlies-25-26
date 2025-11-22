package org.firstinspires.ftc.teamcode.mech.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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


@Autonomous(name="Left Auto")
//ts is the auto for one side you can prolly mirror for the other side
public class LeftAuto extends LinearOpMode{
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

        class RobotMechanisms {
            // 2. Define the method that returns an Action


            Pose2d Startingpos = new Pose2d(-12.52, 12, Math.toRadians(180));
            //ts is all placeholders change it up when gain brainpower

            // ROW 1 (Top) - Moving Leftwards (Center -> Wall)
            Vector2d Ballrow1 = new Vector2d(-39.719, 11.969);
            //Start of row (closer to center)
            Pose2d Ballrow1pose = new Pose2d(-39.719, 11.969,Math.toRadians(180));
            Vector2d Ballrow1_2 = new Vector2d(-44.719, 11.969);
            //Middle of row
            Pose2d Ballrow1_2pose = new Pose2d(-44.719, 11.969,Math.toRadians(180));
            Vector2d Ballrow1_3 = new Vector2d(-49.719, 11.969);
            //Middle of row
            Pose2d Ballrow1_3pose = new Pose2d(-49.719, 11.969,Math.toRadians(180));
            Vector2d Ballrow1end = new Vector2d(-54.719, 11.969);
            //End of row (closer to wall/left)
            Pose2d Ballrow1endpose = new Pose2d(-54.719, 11.969,Math.toRadians(180));

            // ROW 2 (Middle) - Moving Leftwards
            Vector2d Ballrow2 = new Vector2d(-39.719, -11.969);
            //Start of row
            Pose2d Ballrow2pose = new Pose2d(-39.719, -11.969,Math.toRadians(180));
            Vector2d Ballrow2_2 = new Vector2d(-44.719, -11.969);
            //Middle of row
            Pose2d Ballrow2_2pose = new Pose2d(-44.719, -11.969,Math.toRadians(180));
            Vector2d Ballrow2_3 = new Vector2d(-49.719, -11.969);
            //Middle of row
            Pose2d Ballrow2_3pose = new Pose2d(-49.719, -11.969,Math.toRadians(180));
            Vector2d Ballrow2end = new Vector2d(-54.719, -11.969);
            Pose2d Ballrow2endpose = new Pose2d(-54.719, -11.969,Math.toRadians(180));
            //End of row

            // ROW 3 (Bottom) - Moving Leftwards
            Vector2d Ballrow3 = new Vector2d(-39.719, -35.218);
            //Start of row
            Pose2d Ballrow3pose = new Pose2d(-39.719, -35.218,Math.toRadians(180));
            Vector2d Ballrow3_2 = new Vector2d(-44.719, -35.218);
            //Middle of row
            Pose2d Ballrow3_2pose = new Pose2d(-44.719, -35.218,Math.toRadians(180));
            Vector2d Ballrow3_3 = new Vector2d(-49.719, -35.218);
            //Middle of row
            Pose2d Ballrow3_3pose = new Pose2d(-49.719, -35.218,Math.toRadians(180));
            Vector2d Ballrow3end = new Vector2d(-54.719, -35.218);
            //End of row
            Pose2d Ballrow3endpose = new Pose2d(-54.719, -35.218,Math.toRadians(180));

            Vector2d Shootingpos = new Vector2d(-50, 50);
            Pose2d Shootingpospose = new Pose2d(-50, 50,Math.toRadians(135));
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
                            rightIntake.setPower(1);
                            leftIntake.setPower(1);
                            timer = new ElapsedTime();
                            initialized = true;
                        }

                        // 2. Check if 2 seconds have passed
                        if (timer.seconds() < 2.0) {
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
                        if (timer.seconds() < 2.0){
                            return true;
                        }
                        if (timer2 == null) {
                            timer2 = new ElapsedTime();
                            return true;
                        }
                        if (timer.seconds() > 2.0 && timer2.seconds() < 1.0) {
                            leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                            leftFlywheel.setPower(1);
                            rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                            rightFlywheel.setPower(1);
                            return true;
                        }
                        if (timer.seconds() < 2.0){
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

            Action trajectoryCollectRow1_1 = Drivetrain.actionBuilder(Ballrow1pose)
                    .splineTo(Ballrow1_2, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow1_2 = Drivetrain.actionBuilder(Ballrow1_2pose)
                    .splineTo(Ballrow1_3, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow1_3 = Drivetrain.actionBuilder(Ballrow1_3pose)
                    .splineTo(Ballrow1end, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow2_1 = Drivetrain.actionBuilder(Ballrow2pose)
                    .splineTo(Ballrow2_2, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow2_2 = Drivetrain.actionBuilder(Ballrow2_2pose)
                    .splineTo(Ballrow2_3, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow2_3 = Drivetrain.actionBuilder(Ballrow2_3pose)
                    .splineTo(Ballrow2end, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow3_1 = Drivetrain.actionBuilder(Ballrow3pose)
                    .splineTo(Ballrow3_2, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow3_2 = Drivetrain.actionBuilder(Ballrow3_2pose)
                    .splineTo(Ballrow3_3, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow3_3 = Drivetrain.actionBuilder(Ballrow3_3pose)
                    .splineTo(Ballrow3end, Math.toRadians(0))
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
                    trajectoryCollectRow1_3,
                    collectRowOfBalls(2)
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
            Action Trajectoryrow1 = Drivetrain.actionBuilder(Startingpos)
                    .splineTo(Ballrow1,Math.toRadians(0))
                    .build();

            Action Trajectoryrow2 = Drivetrain.actionBuilder(Shootingpospose)
                    .splineTo(Ballrow2,Math.toRadians(0))
                    .build();
            Action Trajectoryrow3 = Drivetrain.actionBuilder(Shootingpospose)
                    .splineTo(Ballrow3,Math.toRadians(0))
                    .build();
            Action Shootingposa1 = Drivetrain.actionBuilder(Ballrow1endpose)
                    .splineToSplineHeading(Shootingpospose,Math.toRadians(135))
                    .build();
            Action Shootingposa2 = Drivetrain.actionBuilder(Ballrow2endpose)
                    .splineTo(Shootingpos,Math.toRadians(135))
                    .build();
            Action Shootingposa3 = Drivetrain.actionBuilder(Ballrow3endpose)
                    .splineTo(Shootingpos,Math.toRadians(135))
                    .build();
            Action autonoumouschain = new SequentialAction(
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
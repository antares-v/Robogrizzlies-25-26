package org.firstinspires.ftc.teamcode.mech.Auto;
import java.util.LinkedHashMap;
import android.os.Build;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mech.Auto.MecanumDrive;



@Autonomous(name="Red Alliance Auto")
//ts is the auto for one side you can prolly mirror for the other side
public class RedAuto extends LinearOpMode{
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


            Pose2d Startingpos = new Pose2d(0, 0, Math.toRadians(0));
            //ts is all placeholders change it up when gain brainpower
            Vector2d Ballrow1 = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 1 for
            Pose2d Ballrow1pose = new Pose2d(0, 0,Math.toRadians(0));
            Vector2d Ballrow1end = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 1 end for
            Pose2d Ballrow1endpose = new Pose2d(0, 0,Math.toRadians(0));
            Vector2d Ballrow2 = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 2 for
            Pose2d Ballrow2pose = new Pose2d(0, 0,Math.toRadians(0));
            Vector2d Ballrow2end = new Vector2d(0, 0);
            Pose2d Ballrow2endpose = new Pose2d(0, 0,Math.toRadians(0));
            //More placeholders for the location of ball rows 2 end for
            Vector2d Ballrow3 = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 3 for
            Pose2d Ballrow3pose = new Pose2d(0, 0,Math.toRadians(0));
            Vector2d Ballrow3end = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 3 end for
            Pose2d Ballrow3endpose = new Pose2d(0, 0,Math.toRadians(0));
            Vector2d Shootingpos = new Vector2d(0, 0);
            Pose2d Shootingpospose = new Pose2d(0, 0,Math.toRadians(0));
            MecanumDrive Drivetrain = new MecanumDrive(hardwareMap, Startingpos);



            public Action collectRowOfBalls() {
                return new Action() {
                    private boolean initialized = false;
                    private ElapsedTime timer;

                    @Override
                    public boolean run(TelemetryPacket packet) {
                        // 1. Start the motors on the very first run
                        if (!initialized) {
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

            Action trajectoryCollectRow1 = Drivetrain.actionBuilder(Ballrow1pose)
                    .splineTo(Ballrow1end, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow2 = Drivetrain.actionBuilder(Ballrow2pose)
                    .splineTo(Ballrow2end, Math.toRadians(0))
                    .build();
            Action trajectoryCollectRow3 = Drivetrain.actionBuilder(Ballrow3pose)
                    .splineTo(Ballrow3end, Math.toRadians(0))
                    .build();

            // 2. Combine the trajectory and your intake action
            Action Pickup1 = new ParallelAction(
                    trajectoryCollectRow1,       // The driving action
                    collectRowOfBalls()    // The intake action (from your previous code)
            );
            Action Pickup2 = new ParallelAction(
                    trajectoryCollectRow2,       // The driving action
                    collectRowOfBalls()    // The intake action (from your previous code)
            );
            Action Pickup3 = new ParallelAction(
                    trajectoryCollectRow3,       // The driving action
                    collectRowOfBalls()    // The intake action (from your previous code)
            );

            Action Trajectoryrow1 = Drivetrain.actionBuilder(Startingpos)
                    .splineTo(Ballrow1,Math.toRadians(0))
                    .build();

            Action Trajectoryrow2 = Drivetrain.actionBuilder(Ballrow1endpose)
                    .splineTo(Ballrow2,Math.toRadians(0))
                    .build();
            Action Trajectoryrow3 = Drivetrain.actionBuilder(Ballrow2endpose)
                    .splineTo(Ballrow3,Math.toRadians(0))
                    .build();
            Action Shootingposa1 = Drivetrain.actionBuilder(Ballrow1endpose)
                    .splineTo(Shootingpos,Math.toRadians(0))
                    .build();
            Action Shootingposa2 = Drivetrain.actionBuilder(Shootingpospose)
                    .splineTo(Shootingpos,Math.toRadians(0))
                    .build();
            Action Shootingposa3 = Drivetrain.actionBuilder(Shootingpospose)
                    .splineTo(Shootingpos,Math.toRadians(0))
                    .build();
            Action autonoumouschain = new SequentialAction(
                    Trajectoryrow1,
                    Pickup1,
                    Shootingposa1,
                    Shoot(),
                    Trajectoryrow2,
                    Pickup2,
                    Shootingposa2,
                    Shoot(),
                    Trajectoryrow3,
                    Pickup3,
                    Shootingposa3,
                    Shoot()
            );
        }

    }
}
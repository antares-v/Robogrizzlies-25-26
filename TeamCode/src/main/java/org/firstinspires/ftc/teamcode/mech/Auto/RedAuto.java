package org.firstinspires.ftc.teamcode.mech.Auto;

import android.os.Build;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mech.Auto.MecanumDrive;



@Autonomous(name="Red Alliance Auto")
//ts is the auto for one side you can prolly mirror for the other side
public class RedAuto extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        class RobotMechanisms {
            // 2. Define the method that returns an Action
            public Action collectRowOfBalls() {
                // 3. The "return telemetry -> { ... };" syntax is a shortcut
                //    for creating a simple, one-off action.
                return telemetry -> {
                    telemetry.addLine("Intake On - Collecting Balls...");
                    sleep(1500); // Wait for 1.5 seconds
                    return false; // This tells the sequence that the action is finished
                };
            }

            public Action shoot() {
                // Example of another action
                return telemetry -> {
                    telemetry.addLine("Shooting...");
                    sleep(1000); // Simulate shooting time
                    return false;
                };

            }

            Pose2d Startingpos = new Pose2d(0, 0, Math.toRadians(0));
            //ts is all placeholders change it up when gain brainpower
            Vector2d Ballrow1 = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 1 for
            Vector2d Ballrow1end = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 1 end for
            Vector2d Ballrow2 = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 2 for
            Vector2d Ballrow2end = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 2 end for
            Vector2d Ballrow3 = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 3 for
            Vector2d Ballrow3end = new Vector2d(0, 0);
            //More placeholders for the location of ball rows 3 end for
            Vector2d Shootingpos = new Vector2d(0, 0);
            MecanumDrive Drivetrain = new MecanumDrive(hardwareMap, Startingpos);

            Action autonoumouschain = Drivetrain.actionBuilder(Startingpos)
                    .splineTo(Ballrow1, Math.toRadians(0))
                    .splineTo(Ballrow2, Math.toRadians(0))
                    .splineTo(Ballrow3, Math.toRadians(0))
                    .build();
        }
}
}
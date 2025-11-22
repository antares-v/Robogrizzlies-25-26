package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 0402.2)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, -60, Math.toRadians(0)))
                        //Far Row
                        .strafeTo(new Vector2d(24, 11))
                        .strafeTo(new Vector2d(38,11))
                        .splineToLinearHeading(new Pose2d(50,50, Math.toRadians(45)), Math.toRadians(45))// Fixed bug from original code
                        .waitSeconds(2)

                        // Middle Row
                        .strafeToLinearHeading(new Vector2d(0, -15), Math.toRadians(0))
                        .strafeTo(new Vector2d(38,-15))
                        .splineToLinearHeading(new Pose2d(50,50, Math.toRadians(45)), Math.toRadians(45))
                        .waitSeconds(2)

                        // Close Row
                        .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(0))
                        .strafeTo(new Vector2d(38,-35))
                        .splineToLinearHeading(new Pose2d(50,50, Math.toRadians(45)), Math.toRadians(45))
                        .waitSeconds(2)

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
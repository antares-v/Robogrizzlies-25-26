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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                        .splineTo(new Vector2d(54.719, 11.969), Math.toRadians(180))
                        .splineTo(new Vector2d(49.719, 11.969), Math.toRadians(180))
                        .splineTo(new Vector2d(44.719, 11.969), Math.toRadians(180)) // Fixed bug from original code
                        .splineTo(new Vector2d(39.719, 11.969), Math.toRadians(180))
                        // Go to shoot
                        .splineTo(new Vector2d(0, 0), Math.toRadians(45))
                        .waitSeconds(2)

                        // --- ROW 2 ---
                        .splineTo(new Vector2d(54.719, -11.969), Math.toRadians(180))
                        .splineTo(new Vector2d(49.719, -11.969), Math.toRadians(180))
                        .splineTo(new Vector2d(44.719, -11.969), Math.toRadians(180))
                        .splineTo(new Vector2d(39.719, -11.969), Math.toRadians(180))
                        // Go to shoot
                        .splineTo(new Vector2d(0, 0), Math.toRadians(45))
                        .waitSeconds(2)

                        // --- ROW 3 ---
                        .splineTo(new Vector2d(54.719, -35.218), Math.toRadians(180))
                        .splineTo(new Vector2d(49.719, -35.218), Math.toRadians(180))
                        .splineTo(new Vector2d(44.719, -35.218), Math.toRadians(180))
                        .splineTo(new Vector2d(39.719, -35.218), Math.toRadians(180))
                        // Go to shoot
                        .splineTo(new Vector2d(0, 0), Math.toRadians(45))
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
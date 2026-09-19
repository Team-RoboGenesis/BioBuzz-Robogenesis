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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(0, 63), Math.toRadians(270)))
                .strafeTo(new Vector2d(0, 0))
                .strafeTo(new Vector2d(10, 0))
                .strafeTo(new Vector2d(10, 10))
                .strafeTo(new Vector2d(-20, 20))
                .strafeToLinearHeading(new Vector2d(20, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, 40), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-40, 40, Math.toRadians(180)), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(30, 30), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(0, 63, Math.toRadians(270)), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
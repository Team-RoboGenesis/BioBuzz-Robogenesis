package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.util.Objects;

import javax.imageio.ImageIO;

public class sampleMeepMeepAuto {
    public static void main(String[] args) {
        Image img = null;
        try {
            img = ImageIO.read(Objects.requireNonNull(
                    MeepMeepTesting.class.getResourceAsStream("/biobuzz.png")
            ));
        } catch (Exception e) {
            e.printStackTrace();
        }


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
                .splineToSplineHeading(new Pose2d(0, -53, Math.toRadians(90)), Math.toRadians(0))
                .build());

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

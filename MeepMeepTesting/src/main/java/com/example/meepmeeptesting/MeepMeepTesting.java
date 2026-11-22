package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Objects;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    // preset positions
    private static double left = Math.toRadians(180);
    private static double right = Math.toRadians(0);
    private static double up = Math.toRadians(90);
    private static double down = Math.toRadians(-90);
    private static Vector2d rightHive = new Vector2d(60, -12);
    private static Vector2d leftHive = new Vector2d(-60, -12);
    private static Vector2d backFlower = new Vector2d(24, -50);
    private static Vector2d rightFlower = new Vector2d(-63, -12);
    private static Vector2d leftFlower = new Vector2d(-63, -12);
    private static Vector2d topFlower = new Vector2d(-63, -12);

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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(60, 0), Math.toRadians(0)))
                .lineToX(40)
                .build());

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
package org.firstinspires.ftc.teamcode.tests;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="JiaAttemptAuto")

    public class JiaAttemptAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d(new Vector2d(0,63),Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();
        Action path = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, 0))
                .splineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(90))
                .build();
        }}


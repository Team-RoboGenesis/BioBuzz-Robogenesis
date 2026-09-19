package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="odometry test auto")
public class OdometryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(new Vector2d(0, 63), Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, 0))
                .strafeTo(new Vector2d(10, 0))
                .strafeTo(new Vector2d(10, 10))
                .strafeTo(new Vector2d(-20, 20))
                .strafeToLinearHeading(new Vector2d(20, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, 40), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-40, 40, Math.toRadians(180)), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(30, 30), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(0, 63, Math.toRadians(270)), Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(path));
    }
}

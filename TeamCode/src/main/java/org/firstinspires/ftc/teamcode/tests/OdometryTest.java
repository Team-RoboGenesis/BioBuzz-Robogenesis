package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="odometry test auto")
public class OdometryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Start at the center of the field facing forward (90 degrees)
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        if (isStopRequested()) return;

        // Drive straight forward 30 inches
        Action path = drive.actionBuilder(beginPose)
                .lineToY(30)
                .build();

        Actions.runBlocking(path);
    }
}

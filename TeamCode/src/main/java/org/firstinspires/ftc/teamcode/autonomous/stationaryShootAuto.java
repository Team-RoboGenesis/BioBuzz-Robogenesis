package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "stationary shoot  auto")
public class stationaryShootAuto extends LinearOpMode {
    // preset positions
    private double left = Math.toRadians(180);
    private double right = Math.toRadians(0);
    private double up = Math.toRadians(90);
    private double down = Math.toRadians(-90);
    private Vector2d rightHive = new Vector2d(60, -12);
    private Vector2d leftHive = new Vector2d(-60, -12);
    private Vector2d backFlower = new Vector2d(24, -50);
    private Vector2d rightFlower = new Vector2d(-63, -12);
    private Vector2d leftFlower = new Vector2d(-63, -12);
    private Vector2d topFlower = new Vector2d(-63, -12);

    @Override
    public void runOpMode() throws InterruptedException {
        subsystems.initAuto(hardwareMap, "blue");
        Pose2d beginPose = new Pose2d(new Vector2d(60, 0), Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Auto pathing
        TrajectoryActionBuilder move = drive.actionBuilder(beginPose)
                // first cycle
                .lineToX(40)
                ;

        Action auto = move.build();

        waitForStart();

        subsystems.startFlywheel();
        subsystems.shoot();
        // Run the path declared above
        Actions.runBlocking(auto);
        subsystems.stopAll();
    }
}

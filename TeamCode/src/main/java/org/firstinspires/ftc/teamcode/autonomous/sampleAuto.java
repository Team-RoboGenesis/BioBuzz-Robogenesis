package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "sample auto")
public class sampleAuto extends LinearOpMode {
    // preset positions
    private double left = Math.toRadians(180);
    private double right = Math.toRadians(0);
    private double up = Math.toRadians(90);
    private double down = Math.toRadians(-90);
    private Vector2d rightHive = new Vector2d(63, -12);
    private Vector2d leftHive = new Vector2d(-63, -12);
    private Vector2d backFlower = new Vector2d(24, -56);
    private Vector2d rightFlower = new Vector2d(-63, -12);
    private Vector2d leftFlower = new Vector2d(-63, -12);
    private Vector2d topFlower = new Vector2d(-63, -12);

    @Override
    public void runOpMode() throws InterruptedException {
        subsystems.initAuto();
        Pose2d beginPose = new Pose2d(0, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Auto pathing
        TrajectoryActionBuilder move = drive.actionBuilder(beginPose)
                // first cycle
                .strafeToLinearHeading(rightHive, left)
                .stopAndAdd(subsystems::shoot)
                // intake from flower
                .strafeToLinearHeading(backFlower, down)
                .stopAndAdd(subsystems::alternateFlowerIntake) // now in DOWN
                .stopAndAdd(subsystems::startIntake)
                .strafeTo(new Vector2d(24, -60))
                .stopAndAdd(() -> subsystems.sleep(2000))
                .strafeTo(backFlower)
                .stopAndAdd(subsystems::alternateFlowerIntake) // now in UP
                .stopAndAdd(() -> subsystems.sleep(2000)) // TODO: temp remove after uncomment
                // move and shoot
//                .strafeToLinearHeading(leftHive, right)
//                .stopAndAdd(subsystems::shoot)
                ;

        Action auto = move.build();

        waitForStart();

        subsystems.startFlywheel();
        // Run the path declared above
        Actions.runBlocking(auto);
        subsystems.stopAll();
    }
}

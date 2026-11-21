package org.firstinspires.ftc.teamcode.tests;
import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TelemetryOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "April tag tracking")
public class aprilTagTest extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        // 1. Initialize the AprilTag processor
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .build();

        // 2. Build the VisionPortal, linking it to your configured webcam name
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cam 1"))
                .setCameraResolution(new Size(1280,720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();
        //@ObjectOverride
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, -53, Math.toRadians(90)));//TODO: GET REAL VALUES
//do values later
        waitForStart();

        while (opModeIsActive()) {
            // 3. Get the list of current AprilTag detections
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // 4. Iterate through detections to pull pose and ID data
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                   telemetry.addLine(String.format("\n==== (ID %d) ====", detection.id));
//
//                    telemetry.addData("XYZ Translation", "X: %6.2f, Y: %6.2f, Z: %6.2f (inches)",
//                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);

                    // To remember:
                    // Range: Straight line distance from camera lens to the tag center
                    // Bearing: Horizontal angle offset to the tag (- degrees is Left, + is Right)
                    // Elevation: Vertical angle offset to the tag
//                    telemetry.addData("Navigation Info", "Range: %6.2f (in), Bearing: %6.2f (deg), Elevation: %6.2f (deg)",
//                            detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);
//
//
//                    telemetry.addData("Relative Angles", "Yaw: %6.2f, Pitch: %6.2f, Roll: %6.2f (deg)",
//                            detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll);
//                    Pose2d poseEstimate = drive.localizer.getPose();
                    

//                } else {
                    telemetry.addLine(String.format("== (ID %d)", detection.id));
                }
            }
            telemetry.update();
            sleep(20);
        }

}}
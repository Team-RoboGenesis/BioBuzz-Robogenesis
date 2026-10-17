package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import android.util.Size;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "blob positioning test")
public class blobPositioningTest extends OpMode{
    // settings
    private final int width = 640;
    private final int height = 480;
    private final int gain = 25;
    private final int exposureTimeMs = 5; // lower is required for high FPS
    private final int telemetryDataLimit = 10; // reduce to improve performance/screen cluttering with many blobs
    private final int FOV = 70;
    private final double nectarAreaAt20Cm = 4500;
    private final double pollenAreaAt20Cm = 10;

    // default value settings
    private int dataCount = 0;
    private int iterations = 0;
    private boolean controlsOptimized = false;

    // devices
//    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    // functional
    private VisionPortal visionPortal1;
    private VisionPortal visionPortal2;
    private ColorBlobLocatorProcessor cam1RedBlobProcessor;
    private ColorBlobLocatorProcessor cam1BlueBlobProcessor;
    private ColorBlobLocatorProcessor cam1YellowBlobProcessor;
    private ColorBlobLocatorProcessor cam2RedBlobProcessor;
    private ColorBlobLocatorProcessor cam2BlueBlobProcessor;
    private ColorBlobLocatorProcessor cam2YellowBlobProcessor;
    private Map<String, List<ColorBlobLocatorProcessor.Blob>> blobs = new HashMap<>(); // haha, funny data type name
    private Map<String, Double> blobAreas = new HashMap<>();
    private ExposureControl exposureControl;
    private GainControl gainControl;

    private ColorBlobLocatorProcessor createProcessor(ColorRange range) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(range)
                .setContourMode(
                        ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY
                )
                .setBlurSize(5)
                .setMorphOperationType(
                        ColorBlobLocatorProcessor.MorphOperationType.CLOSING
                )
                .setErodeSize(5)
                .setDilateSize(5)
                .setDrawContours(true)
                .build();
    }

    @Override
    public void init() {
        blobAreas.put("red blob", nectarAreaAt20Cm);
        blobAreas.put("blue blob", nectarAreaAt20Cm);
        blobAreas.put("yellow blob", pollenAreaAt20Cm);
        // setup camera and processors
        // 1. Build the ColorBlobLocator Processors
        ColorRange red = new ColorRange(
                ColorSpace.HSV,
                new Scalar(0, 45, 45),
                new Scalar(18, 255, 255)
        );

        ColorRange blue = new ColorRange(
                ColorSpace.HSV,
                new Scalar(95, 80, 65),
                new Scalar(135, 255, 255)
        );

        ColorRange yellow = new ColorRange(
                ColorSpace.HSV,
                new Scalar(18, 80, 85),
                new Scalar(38, 255, 255)
        );


        cam1RedBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(red)
                .setContourMode(
                        ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY
                )
                .setBlurSize(3)
                .setDrawContours(true)
                .build();
        cam1BlueBlobProcessor = createProcessor(blue);
        cam1YellowBlobProcessor = createProcessor(yellow);

        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "cam 1");


        // 2. Build the VisionPortal and attach all processors
        visionPortal1 = new VisionPortal.Builder()
                .setCamera(webcam1)
                .addProcessors(cam1RedBlobProcessor/*, cam1BlueBlobProcessor, cam1YellowBlobProcessor*/)
                .setCameraResolution(new Size(width, height)) // Standard resolution for high FPS tracking
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Mandatory for high-FPS global shutter webcams
                .build();



//        cam2RedBlobProcessor = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(red)
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .build();
//
//        cam2BlueBlobProcessor = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(blue)
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .build();
//
//        cam2YellowBlobProcessor = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(yellow)
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .build();
//        WebcamName webcam2 = hardwareMap.get(WebcamName.class, "cam 2");


//        visionPortal2 = new VisionPortal.Builder()
//                .setCamera(webcam2)
//                .addProcessors(redBlobProcessor, blueBlobProcessor, yellowBlobProcessor)
//                .setCameraResolution(new Size(width, height)) // Standard resolution for high FPS tracking
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Mandatory for high-FPS global shutter webcams
//                .build();
//

        telemetry.addLine("initialized and ready to start!");
        telemetry.update();
    }

    // runs after init but before loop (I didn't know this existed either)
    @Override
    public void init_loop() {
        if (!controlsOptimized && visionPortal1.getCameraState() == VisionPortal.CameraState.STREAMING/*&& visionPortal2.getCameraState() == VisionPortal.CameraState.STREAMING*/) { // remove comment for multiple cameras
            // Retrieve control objects
            exposureControl = visionPortal1.getCameraControl(ExposureControl.class);
            gainControl = visionPortal1.getCameraControl(GainControl.class);

            // Switch exposure mode to Manual to enable tuning
            exposureControl.setMode(ExposureControl.Mode.Manual);

            // note: Low exposure time reduces motion blur at high FPS
            // Pair lower exposure with a higher gain to maintain picture brightness
            exposureControl.setExposure(exposureTimeMs, TimeUnit.MILLISECONDS);
            gainControl.setGain(gain);
//
//             // Retrieve control objects
//             exposureControl = visionPortal2.getCameraControl(ExposureControl.class);
//             gainControl = visionPortal2.getCameraControl(GainControl.class);
//
//             // Switch exposure mode to Manual to enable tuning
//             exposureControl.setMode(ExposureControl.Mode.Manual);
//
//             // Global Shutter Strategy: Low exposure time reduces motion blur at high FPS
//             // Pair lower exposure with a higher gain to maintain picture brightness
//             exposureControl.setExposure(exposureTimeMs, TimeUnit.MILLISECONDS);
//             gainControl.setGain(gain);
//
            controlsOptimized = true;
        }

        if (controlsOptimized) {
            telemetry.addLine("Camera controls applied! Ready to start.");
        } else {
            telemetry.addLine("Waiting for camera stream to open...");
        }
        telemetry.update();
    }


    @Override
    public void loop() {
//        // 1. Tell the localizer to read the encoder/IMU sensors
//        drive.localizer.update();
//
//        // 2. Extract the current position directly from the localizer
//        Pose2d currentPose = drive.localizer.getPose();
        Pose2d currentPose = new Pose2d(0, 0, 0);

        // 3. Log or use the pose coordinates
        double botX = currentPose.position.x;
        double botY = currentPose.position.y;
        double botHeading = Math.toDegrees(currentPose.heading.toDouble());

        telemetry.update();

        dataCount = 0;

        // add blobs to the dict
        blobs.put("red blob",  cam1RedBlobProcessor.getBlobs());
//        blobs.put("blue blob",  cam1BlueBlobProcessor.getBlobs());
//        blobs.put("yellow blob",  cam1YellowBlobProcessor.getBlobs());

        for (List<ColorBlobLocatorProcessor.Blob> blobList : blobs.values()) {
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 500, 300000, blobList
            );

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 1.6, blobList
            );
        }

        // loop through every color in the dict
        for (Map.Entry<String, List<ColorBlobLocatorProcessor.Blob>> blob : blobs.entrySet()) {
            // limit telemetry data
            if (dataCount >= telemetryDataLimit) {
                break;
            }
            iterations = 0;
            // loop through every blob in the color's list
            for (ColorBlobLocatorProcessor.Blob currentBlob : blob.getValue()) {
                String telemetryStr = blob.getKey(); // make the start of the line ([color] blob )
                // limit telemetry data
                if (dataCount >= telemetryDataLimit) {
                    break;
                }
                telemetryStr = telemetryStr + " " + iterations; // add the iteration number ([color] blob [iteration])
                // Get the bounding box
                RotatedRect boxFit = currentBlob.getBoxFit();
                // convert to non-rotated rect
                Rect rect = boxFit.boundingRect();

                // get data
                int width  = rect.width;
                int height = rect.height;

                double centerX = boxFit.center.x;
                double centerY = boxFit.center.y;

                double area = currentBlob.getContourArea();;

                // offset X based on camera origin shifting
                double pixelOffset = centerX - (width / 2.0);
                double fovRadians = Math.toRadians(FOV);
                double angleRadians = Math.atan((2.0 * pixelOffset / width) * Math.tan(fovRadians / 2.0)); // get final angle (in radians) of the blob
                double blobHeading = Math.toDegrees(angleRadians) + botHeading; // convert to degrees and add bot heading offset

                double blobDistance = 20 * Math.sqrt(blobAreas.get(blob.getKey()) / area);
                double blobX = Math.floor(botX + blobDistance * Math.cos(Math.toRadians(blobHeading)));
                double blobY = Math.floor(botY + blobDistance * Math.sin(Math.toRadians(blobHeading)));

                String dataStr = "x: " + blobX + ", y: " + blobY;

                telemetry.addData(telemetryStr, dataStr);

                dataCount++;
                iterations++;
            }
        }


        if (dataCount == 0) {
            // fallback in case no blobs are detected
            telemetry.addLine("no blobs were detected!");
        }

        telemetry.update();

        blobs.clear();
    }
}

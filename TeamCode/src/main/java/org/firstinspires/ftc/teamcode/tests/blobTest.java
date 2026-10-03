package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

import android.util.Size;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "blob test")
public class blobTest extends OpMode {
    // settings
    private int width = 640;
    private int height = 480;
    private int gain = 25;
    private int exposureTimeMs = 5;
    private int telemetryDataLimit = 10;

    // formatting
    private int dataCount = 0;
    private int iterations = 0;
    private boolean controlsOptimized = false;

    // functional
    private VisionPortal visionPortal1;
    private VisionPortal visionPortal2;
    private ColorBlobLocatorProcessor cam1RedBlobProcessor;
    private ColorBlobLocatorProcessor cam1BlueBlobProcessor;
    private ColorBlobLocatorProcessor cam1YellowBlobProcessor;
    private ColorBlobLocatorProcessor cam2RedBlobProcessor;
    private ColorBlobLocatorProcessor cam2BlueBlobProcessor;
    private ColorBlobLocatorProcessor cam2YellowBlobProcessor;
    private ExposureControl exposureControl;
    private GainControl gainControl;



    @Override
    public void init() {
        // setup camera and processors
        // 1. Build the ColorBlobLocator Processors
        cam1RedBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .build();

        cam1BlueBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .build();

        cam1YellowBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .build();

        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "cam 1");


        // 2. Build the VisionPortal and attach all processors
        visionPortal1 = new VisionPortal.Builder()
                .setCamera(webcam1)
                .addProcessors(cam1RedBlobProcessor, cam1BlueBlobProcessor, cam1YellowBlobProcessor)
                .setCameraResolution(new Size(width, height)) // Standard resolution for high FPS tracking
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Mandatory for high-FPS global shutter webcams
                .build();


//        cam2RedBlobProcessor = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.RED)
//                .build();
//
//        cam2BlueBlobProcessor = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.BLUE)
//                .build();
//
//        cam2YellowBlobProcessor = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.YELLOW)
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
        dataCount = 0;
        Map<String, List<ColorBlobLocatorProcessor.Blob>> blobs = new HashMap<>();

        // add blobs to the dict
        blobs.put("red blobs",  cam1RedBlobProcessor.getBlobs());
        blobs.put("blue blobs",  cam1BlueBlobProcessor.getBlobs());
        blobs.put("yellow blobs",  cam1YellowBlobProcessor.getBlobs());

        // loop through every color in the dict
        for (Map.Entry<String, List<ColorBlobLocatorProcessor.Blob>> blob : blobs.entrySet()) {
            iterations = 0;
            String telemetryStr = blob.getKey() + " blob "; // make the start of the line ([color] blob )
            // loop through every blob in the color's list
            for (ColorBlobLocatorProcessor.Blob currentBlob : blob.getValue()) {
                telemetryStr = telemetryStr + iterations; // add the iteration number ([color] blob [iteration])
                // Get the bounding box
                RotatedRect boxFit = currentBlob.getBoxFit();
                // convert to non-rotated rect
                Rect rect = boxFit.boundingRect();

                // get data
                int width  = rect.width;
                int height = rect.height;

                double centerX = boxFit.center.x;
                double centerY = boxFit.center.y;

                // make it one concise string
                String dataStr = "x: " + centerX + " y: " + centerY;
                dataStr = dataStr + " width: " + width + " height: " + height;
                dataStr = dataStr + " area: " + width * height;

                // limit for telemetry data
                if (dataCount <= telemetryDataLimit) {
                    telemetry.addData(telemetryStr, dataStr);
                }
                dataCount++;
                iterations++;
            }
        }


        if (dataCount == 0) {
            // fallback in case no blobs are detected
            telemetry.addLine("no blobs were detected!");
        }

        telemetry.update();
    }
}

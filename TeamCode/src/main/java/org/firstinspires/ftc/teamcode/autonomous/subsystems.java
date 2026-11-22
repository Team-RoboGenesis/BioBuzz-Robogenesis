package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PIDF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class subsystems {
    // constants
    private static final double ON = 1;
    private static final double OFF = 0;
    private static final double RED = 0.280;
    private static final double BLUE = 0.611;
    private static final double YELLOW = 0.388;
    private static final double INTAKEPOWER = 1;
    private static final double MOTORTICKSPERORTATION = 28;
    private static final double TICKS_PER_TURRET_ROTATION = 1436;
    private static final double shootingRPM = 3450; // TODO: get real value
    private static final double armDownPos = 0.41;
    private static final double armUpPos = 0;
    private static final double trasferServoSpeed = 1;
    private static final double startX = 0;
    private static final double startY = -53;

    // device setup
    private static PIDF flywheelPidf;
    private static PIDF turretBasePid;
    private static AprilTagProcessor aprilTag;
    private static MecanumDrive drive = null;
    private static DcMotorEx rightFront = null;
    private static DcMotor leftFront = null;
    private static DcMotor rightBack = null;
    private static DcMotor leftBack = null;
    private static DcMotor intake = null;
    private static DcMotorEx turretFlywheel = null;
    private static IMU imu = null;
    private static CRServo turretBase = null;
    private static Servo rightFlowerIntake = null;
    private static Servo leftFlowerIntake = null;
    private static CRServo rightTransferServo = null;
    private static CRServo leftTransferServo = null;

    // starting values
    private static boolean flywheelPidfEnabled = true;
    private static double lastTime = System.nanoTime();
    private static double motorCurrentRPM = 0;
    private static List<Integer> aprilTags = new ArrayList<>();

    public static void sleep(double millis) {
        try {
            Thread.sleep((long) millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public static void initAuto(HardwareMap hardwareMap, String color) {
        if (color == "blue") {
            aprilTags.addAll(List.of(42, 43, 44, 45, 38, 39, 40, 41));
        } else {
            aprilTags.addAll(List.of(30, 31, 32, 33, 34, 35, 36, 37));
        }
        // make motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turretFlywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                // Note: if these values are incorrect, then the IMU
                // will return an inaccurate heading
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // april tag
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cam 1"))
                .setCameraResolution(new Size(1280,720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        // Initialize MecanumDrive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, -53, Math.toRadians(90))); // TODO: get real values

        // make servos
        turretBase = hardwareMap.get(CRServo.class, "turretBase");
        turretBase.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlowerIntake = hardwareMap.get(Servo.class, "rightFlowerIntake");
        leftFlowerIntake = hardwareMap.get(Servo.class, "leftFlowerIntake");
        rightTransferServo = hardwareMap.get(CRServo.class, "rightTransferServo");
        leftTransferServo = hardwareMap.get(CRServo.class, "leftTransferServo");

        // make sensors
//        distIn = new GateSensor(hardwareMap, "distIn", distInThreshold, true);
//        distOut = new GateSensor(hardwareMap,"distOut", distOutThreshold, false);
//        distanceSensors.add(distIn);
//        distanceSensors.add(distOut);

        // make PIDFs
        // make flywheel PIDF
        double p = 0.000007;
        double i = 0;
        double d = 0;
        double f = 0.0002002;
        flywheelPidf = new PIDF(p, i, d, f);

        // make turret angle PID
        p = 0.005;
        i = 0;
        d = 0;
        turretBasePid = new PIDF(p, i, d, 0);

//        // make LED's
//        leds.add(hardwareMap.get(Servo.class, "led1"));
//        leds.add(hardwareMap.get(Servo.class, "led2"));
//        leds.add(hardwareMap.get(Servo.class, "led3"));
//        leds.add(hardwareMap.get(Servo.class, "led4"));
    }

    public static boolean getUp() {
        // 3. Get the list of current AprilTag detections
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<Integer> detectedIDs = new ArrayList<>();

        // 4. Iterate through detections to pull pose and ID data
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                detectedIDs.add(detection.id);
            }
        }
        return !Collections.disjoint(detectedIDs, aprilTags); // get if any of the IDs are in the april tag list
    }

    public static void startFlywheel() {
        while (shootingRPM - motorCurrentRPM > 100) {
            double currentTime = System.nanoTime();
            double dt = (currentTime - lastTime) / 1_000_000_000; // nanoseconds -> seconds
            dt = Math.max(0.000001, dt); // avoid division by 0
            motorCurrentRPM = turretFlywheel.getVelocity() / MOTORTICKSPERORTATION * 60.0;
            double motorPower = flywheelPidf.tick(shootingRPM, motorCurrentRPM, dt);
            motorPower = Math.max(OFF, Math.min(ON, motorPower));
            turretFlywheel.setPower(motorPower);
            lastTime = currentTime;
            sleep(10);
        }
        sleep(2000);
    }

    public static void startIntake() {
        intake.setPower(INTAKEPOWER);
    }

    public static void alternateFlowerIntake() {
        if (rightFlowerIntake.getPosition() == armUpPos) {
            rightFlowerIntake.setPosition(armDownPos);
            leftFlowerIntake.setPosition(armDownPos);
        } else {
            rightFlowerIntake.setPosition(armUpPos);
            leftFlowerIntake.setPosition(armUpPos);
        }
    }

    public static void shoot() {
        intake.setPower(INTAKEPOWER);
        rightTransferServo.setPower(trasferServoSpeed);
        leftTransferServo.setPower(-trasferServoSpeed);
        sleep(4000);
        rightTransferServo.setPower(OFF);
        leftTransferServo.setPower(OFF);
    }

    public static void stopAll() {
        intake.setPower(OFF);
        turretFlywheel.setPower(OFF);
        rightTransferServo.setPower(OFF);
        leftTransferServo.setPower(OFF);
        rightFlowerIntake.setPosition(armUpPos);
        leftFlowerIntake.setPosition(armUpPos);
    }
}

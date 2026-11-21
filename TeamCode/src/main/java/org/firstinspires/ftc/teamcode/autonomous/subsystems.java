package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PIDF;
import org.firstinspires.ftc.teamcode.tests.fileUtil;

import java.util.List;

public class subsystems {
    // constants
    private final double ON = 1;
    private final double OFF = 0;
    private final double RED = 0.280;
    private final double BLUE = 0.611;
    private final double YELLOW = 0.388;
    private final double INTAKEPOWER = 1;
    private final double MOTORTICKSPERORTATION = 28;
    private final double TICKS_PER_TURRET_ROTATION = 1436;
    private final double shootingRPM = 3550; // TODO: get real value
    private final double armDownPos = 0.41;
    private final double armUpPos = 0;
    private final double trasferServoSpeed = 1;
    private final double startX = 0;
    private final double startY = -53;

    // device setup
    private PIDF flywheelPidf;
    private PIDF turretBasePid;
    private MecanumDrive drive = null;
    private DcMotorEx rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private DcMotor intake = null;
    private DcMotorEx turretFlywheel = null;
    private IMU imu = null;
    private CRServo turretBase = null;
    private Servo rightFlowerIntake = null;
    private Servo leftFlowerIntake = null;
    private CRServo rightTransferServo = null;
    private CRServo leftTransferServo = null;

    // starting values
    private boolean flywheelPidfEnabled = true;
    private double lastTime = System.nanoTime();
    private double motorCurrentRPM = 0;

    public void sleep(double millis) {
        try {
            Thread.sleep((long) millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void init() {
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

    public void startFlywheel() {
        while (shootingRPM - motorCurrentRPM < 100) {
            double currentTime = System.nanoTime();
            double dt = (currentTime - lastTime) / 1_000_000_000; // nanoseconds -> seconds
            dt = Math.max(0.000001, dt); // avoid division by 0
            motorCurrentRPM = turretFlywheel.getVelocity() / MOTORTICKSPERORTATION * 60.0;
            double motorPower = flywheelPidf.tick(shootingRPM, motorCurrentRPM, dt);
            motorPower = Math.max(OFF, Math.min(ON, motorPower));
            turretFlywheel.setPower(motorPower);
            lastTime = currentTime;
        }
    }

    public void startIntake() {
        intake.setPower(INTAKEPOWER);
    }

    public void alternateFlowerIntake() {
        if (rightFlowerIntake.getPosition() == armUpPos) {
            rightFlowerIntake.setPosition(armDownPos);
            leftFlowerIntake.setPosition(armDownPos);
        } else {
            rightFlowerIntake.setPosition(armUpPos);
            leftFlowerIntake.setPosition(armUpPos);
        }
    }

    public void shoot() {
        rightTransferServo.setPower(trasferServoSpeed);
        leftTransferServo.setPower(trasferServoSpeed);
        sleep(4000);
        rightTransferServo.setPower(OFF);
        leftTransferServo.setPower(OFF);
    }

    public void stopAll() {
        intake.setPower(OFF);
        turretFlywheel.setPower(OFF);
        rightTransferServo.setPower(OFF);
        leftTransferServo.setPower(OFF);
        rightFlowerIntake.setPosition(armUpPos);
        leftFlowerIntake.setPosition(armUpPos);
    }
}

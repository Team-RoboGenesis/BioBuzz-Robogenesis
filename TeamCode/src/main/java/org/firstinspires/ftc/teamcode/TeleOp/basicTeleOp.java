package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;
import android.os.Build;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.subsystems;
import org.firstinspires.ftc.teamcode.subsystems.GateSensor;
import org.firstinspires.ftc.teamcode.subsystems.PIDF;
import org.firstinspires.ftc.teamcode.tests.fileUtil;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@TeleOp(name = "basic_TeleOp11/28DAFIFTH")
public class basicTeleOp extends OpMode {
    // constants
    private final double OFF = 0;
    private final double RED = 0.278;
    private final double BLUE = 0.611;
    private final double YELLOW = 0.388;
    private final double ON = 1;
    private final double INTAKEPOWER = 1;
    private final double MOTORTICKSPERORTATION = 28;
    private final double TICKS_PER_TURRET_ROTATION = 1436;
    private final double shootingRPM = 3225; // TODO: get real value
    //Old value 3450
    private final double armDownPos = 0.41;
    private final double armUpPos = 0;
    private final double trasferServoSpeed = 1;
    private final double startX = 0;
    private final double startY = -53;

    // config
    private final double distInThreshold = 10; // TODO: get real value
    private final double distOutThreshold = 10; // TODO: get real value
    private final double lowerTurretLimit = -140;
    private final double upperTurretLimit = 140;

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
    private GateSensor distIn;
    private GateSensor distOut;
    private VoltageSensor voltageSensor = null;
    private List<Servo> leds = new ArrayList<>();
    private List<Double> elements = new ArrayList<>();
    private List<GateSensor> distanceSensors = new ArrayList<>();

    // starting values
    private List<Double> closest = new ArrayList<>();
    private Map<Double, List<Double>> points = new HashMap<>();
    private List<Double> RPMs = new ArrayList<>();
    private List<Double> motorPowers = new ArrayList<>();
    private List<Double> voltages = new ArrayList<>();
    private List<Double> targetRPMs = new ArrayList<>();
    private List<Double> currentAngles = new ArrayList<>();
    private List<Double> servoPowers = new ArrayList<>();
    private List<Double> targetAngles = new ArrayList<>();
    private List<Double> robotXPositions = new ArrayList<>();
    private List<Double> robotYPositions = new ArrayList<>();
    private List<Double> robotHeadings = new ArrayList<>();
    private List<Double> RRHeadings = new ArrayList<>();
    private List<Double> intakeRPMs = new ArrayList<>();
    private List<Double> intakePowers = new ArrayList<>();
    private List<Double> leftFlowerPositions = new ArrayList<>();
    private List<Double> rightFlowerPositions = new ArrayList<>();
    private List<Double> times = new ArrayList<>();
    private List<List<Double>> data = new ArrayList<>();
    private boolean lastAPress = false;
    private double targetRPM = 0;
    private double targetTicksPerMs = 0;
    private double currentSecond = 0;
    private double startingTime = (double) System.nanoTime() / 1_000_000_000;
    private double lastTime = System.nanoTime();
    private double currentTime = System.nanoTime();
    private double motorCurrentTicksPerMs = 0;
    private double motorCurrentRPM = 0;
    private double turretCurrentAngle = 0;
    private double turretLastAngle = 0;
    private double turretTargetAngle = 0;
    private int startServoTicks = 0;
    private boolean flywheelPidfEnabled = false;
    private double iterations = 0;
    private String formattedTime = null;

    public List<Double> getClosestPoint(double value) {
        List<Double> xValues = new ArrayList<>(points.keySet());
        if (xValues.isEmpty()) {
            throw new IllegalArgumentException("List cannot be empty");
        }

        double closest = xValues.get(0);
        double minDiff = Math.abs((long) closest - value); // Use long to prevent int overflow

        for (double num : xValues) {
            double diff = Math.abs(num - value);
            if (diff < minDiff) {
                minDiff = diff;
                closest = num;
            }
        }
        return points.get(closest);
    }

    @Override
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

        // data collection stuffs remember that we get this data every run of loop
        data.add(times);
        data.add(RPMs);
        data.add(motorPowers);
        data.add(targetRPMs);
        data.add(currentAngles);
        data.add(servoPowers);
        data.add(targetAngles);
        data.add(robotXPositions);
        data.add(robotYPositions);
        data.add(robotHeadings);
        data.add(RRHeadings);
        data.add(intakeRPMs);
        data.add(intakePowers);
        data.add(rightFlowerPositions);
        data.add(leftFlowerPositions);
        data.add(voltages);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        SimpleDateFormat formatter = new SimpleDateFormat("HH:mm:ss", Locale.US);
        formattedTime = formatter.format(new Date());
        fileUtil.init(hardwareMap.appContext, "log " + formattedTime);

        startServoTicks = rightFront.getCurrentPosition();

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
        double p = 0.0007;
        double i = 0;
        double d = 0;
        double f = 0.0002002;
        flywheelPidf = new PIDF(p, i, d, f);

        // make turret angle PID
        p = 0.005;
        i = 0;
        d = 0;
        turretBasePid = new PIDF(p, i, d, 0);

        // make hive centers
        // TODO: add real values
        double leftHiveX = 12, leftHiveY = 0;
        double rightHiveX = -12, rightHiveY = 0;

        points.put(leftHiveX, List.of(leftHiveX, leftHiveY));
        points.put(rightHiveX, List.of(rightHiveX, rightHiveY));


//        // make LED's
//        leds.add(hardwareMap.get(Servo.class, "led1"));
//        leds.add(hardwareMap.get(Servo.class, "led2"));
//        leds.add(hardwareMap.get(Servo.class, "led3"));
//        leds.add(hardwareMap.get(Servo.class, "led4"));
    }

    @Override
    public void loop() {
        // element counting with funny class
//        for (GateSensor distSen : distanceSensors) {
//            int data = distSen.getData();
//            if (data == 1) {
//                elements.add(distSen.getColor());
//            } else if (data == -1) {
//                if (!elements.isEmpty()) {
//                    elements.remove(0);
//                }
//            }
//        }

        // led color setting
//        for (Servo led : leds) {
//            led.setPosition(OFF);
//        }
//
//        int i = 0;
//        for (double elementColor : elements) {
//            leds.get(i).setPosition(elementColor);
//            i++;
//        }

        // element check
//        if (elements.size() == 4) {
//            intake.setPower(OFF);
////            flowerIntake1.setPosition(armUpPos);
////            flowerIntake2.setPosition(armUpPos);
//        } else if (elements.size() > 4) {
//            // TODO: launch excess
//            targetRPM = shootingRPM;
//        }

        // reset IMU
        if (gamepad1.options)
        {
            imu.resetYaw();
            drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, Math.toRadians(90)));
        }

        // Roadrunner pos tracking
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Get the IMU angle to counteract it
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Start the drive
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        // Print pose to telemetry
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading", Math.toDegrees(pose.heading.toDouble()));

        // button mapping
        // on press toggle power levels
        if (lastAPress != gamepad2.a) {
            if (gamepad2.a) {
                if (intake.getPower() == 0) {
                    intake.setPower(INTAKEPOWER);
                } else {
                    intake.setPower(OFF);
                }
            }
            lastAPress = gamepad2.a;
        }

        // other inputs
        if (gamepad2.right_bumper) {
            targetRPM = shootingRPM;
            flywheelPidfEnabled = true;
        } else if (gamepad2.y) {
            rightTransferServo.setPower(trasferServoSpeed);
            leftTransferServo.setPower(-trasferServoSpeed);
        } else if (gamepad2.left_bumper) {
            flywheelPidfEnabled = false;
        } else if (gamepad2.dpad_down) {
            rightFlowerIntake.setPosition(armDownPos);
            leftFlowerIntake.setPosition(armDownPos);
        } else if (gamepad2.dpad_up) {
            rightFlowerIntake.setPosition(armUpPos);
            leftFlowerIntake.setPosition(armUpPos);
        } else if (gamepad2.right_stick_button) {
            String dataString = "";
            for (List<Double> point : data) {
                dataString += "\n" + point.stream()
                        .map(String::valueOf)
                        .collect(Collectors.joining(","));
            }
            fileUtil.overwriteFromIndex(dataString, 0);
            telemetry.addLine("loaded file: " + formattedTime);
            telemetry.addData("samples", times.size());
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            telemetry.update();
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        } else if (gamepad2.b) {
            intake.setPower(-INTAKEPOWER);
        } else if (!gamepad2.y) {
            rightTransferServo.setPower(OFF);
            leftTransferServo.setPower(OFF);
        }

//        turretTargetAngle -= gamepad2.right_stick_y / 2;

        // tick PIDF
        closest = getClosestPoint(pose.position.x); // get closest hive position based on X position
        double goalX = closest.get(0);
        double goalY = closest.get(1);

        telemetry.addData("closest gaol x", goalX);
        telemetry.addData("closest gaol y", goalY);

        double robotHeading = pose.heading.toDouble();

        double angleToTarget = Math.atan2(
                goalY - pose.position.y,
                goalX - pose.position.x
        );

//        turretTargetAngle = angleToTarget - robotHeading;
//
//        turretTargetAngle = Math.atan2(
//                Math.sin(turretTargetAngle),
//                Math.cos(turretTargetAngle)
//        );
//
//        turretTargetAngle = Math.toDegrees(turretTargetAngle);

        telemetry.addData("target angle", turretTargetAngle);

        currentTime = System.nanoTime(); // the millisecond version is inaccurate for some reason, so we use nanoseconds
        double dt = (currentTime - lastTime) / 1_000_000_000; // nanoseconds -> seconds
        dt = Math.max(0.000001, dt); // avoid division by 0

        motorCurrentRPM = turretFlywheel.getVelocity() / MOTORTICKSPERORTATION * 60.0;
        telemetry.addData("current RPM", motorCurrentRPM);
        int currentTicks = rightFront.getCurrentPosition() - startServoTicks;
        turretCurrentAngle = (currentTicks / TICKS_PER_TURRET_ROTATION) * 360.0;
        telemetry.addData("current angle", turretCurrentAngle);

//        double servoPower = turretBasePid.tick(turretTargetAngle, turretCurrentAngle, dt);
//        if (Math.abs(servoPower) < 0.25 && !(Math.abs(servoPower) < 0.03)) {
//            servoPower = 0.25 * Math.signum(servoPower);
//        }
//        servoPower = Math.max(-ON, Math.min(ON, servoPower));
//        turretBase.setPower(servoPower);

        double motorPower = 0;
        if (flywheelPidfEnabled) {
            motorPower = flywheelPidf.tick(targetRPM, motorCurrentRPM, dt);
            telemetry.addData("attempted power", motorPower);
            motorPower = Math.max(OFF, Math.min(ON, motorPower));
            turretFlywheel.setPower(motorPower);
            telemetry.addData("target RPM", targetRPM);
            telemetry.addData("current power", motorPower);
        } else {
            turretFlywheel.setPower(OFF);
        }

        if (iterations % 10 == 0) {
            RPMs.add(motorCurrentRPM);
            motorPowers.add(motorPower);
            voltages.add(voltageSensor.getVoltage());
            targetRPMs.add(targetRPM);
            currentAngles.add(turretCurrentAngle);
            servoPowers.add(0.0);
            targetAngles.add(turretTargetAngle);
            robotXPositions.add(pose.position.x);
            robotYPositions.add(pose.position.y);
            robotHeadings.add(Math.toDegrees(robotHeading));
            RRHeadings.add(Math.toDegrees(pose.heading.toDouble()));
            intakePowers.add(intake.getPower());
            rightFlowerPositions.add(rightFlowerIntake.getPosition());
            leftFlowerPositions.add(leftFlowerIntake.getPosition());
            currentSecond = currentTime / 1_000_000_000 - startingTime;
            times.add(currentSecond);
        }

        telemetry.addData("current second", currentSecond);

        lastTime = currentTime;
        iterations++;
        telemetry.update();
    }
}
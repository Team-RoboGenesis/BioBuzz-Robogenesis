package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "basic TeleOp")
public class basicTeleOp extends OpMode {
    // constants
    private final double OFF = 0;
    private final double RED = 0.278;
    private final double BLUE = 0.611;
    private final double YELLOW = 0.388;
    private final double ON = 1;
    private final double INTAKEPOWER = 1;
    private final double MOTORTICKSPERORTATION = 28;
    private final double SERVOTICKSPERORTATION = 4096;
    private final double shootingRPM = 3550; // TODO: get real value
    private final double armDownPos = 0.41;
    private final double armUpPos = 0;
    private final double trasferServoSpeed = 1;
    private final double startX = 0;
    private final double startY = -53;

    // config
    private final double distInThreshold = 10; // TODO: get real value
    private final double distOutThreshold = 10; // TODO: get real value
    private final double upperTurretLimit = 360;
    private final double lowerTurretLimit = 0;

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
    private List<Servo> leds = new ArrayList<>();
    private List<Double> elements = new ArrayList<>();
    private List<GateSensor> distanceSensors = new ArrayList<>();

    // starting values
    private List<Double> closest = new ArrayList<>();
    private Map<Double, List<Double>> points = new HashMap<>();
    private boolean lastAPress = false;
    private double targetRPM = 0;
    private double targetTicksPerMs = 0;
    private double lastTime = System.nanoTime();
    private double currentTime = System.nanoTime();
    private double motorCurrentTicksPerMs = 0;
    private double motorCurrentRPM = 0;
    private double turretCurrentAngle = 0;
    private double turretLastAngle = 0;
    private double turretTargetAngle = 0;
    private boolean flywheelPidfEnabled = false;

    class GateSensor {
        private DistanceSensor distanceSensor;
        private double threshold;
        private boolean last;
        private boolean entrance;
        private NormalizedColorSensor colorSensor;
        private boolean color;

        public GateSensor(HardwareMap hwMap, String deviceName, double threshold, boolean color) {
            this.distanceSensor = hwMap.get(DistanceSensor.class, deviceName);
            this.color = color;
            if (color) {
                this.colorSensor = hwMap.get(NormalizedColorSensor.class, deviceName);
            }
            this.threshold = threshold;
            this.last = false;
            this.entrance = deviceName.toLowerCase().contains("in"); // kinda dumb but it removes an argument
        }

        public int getData() {
            double distance = this.distanceSensor.getDistance(DistanceUnit.CM);
            boolean current = distance < this.threshold;

            if (current != this.last) {
                this.last = current;

                if (this.entrance && current) {
                    return 1;
                } else if (!current) {
                    return -1;
                }
            }

            return 0;
        }

        public double getColor() {
            if (!color) {
                return 0;
            }

            NormalizedRGBA colors = this.colorSensor.getNormalizedColors();
            // get colors with colors.red, green and blue

            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            double hue = hsvValues[0];
            return hue / 360;
        }
    }

    class PIDF {
        public double p, i, d, f;
        private double integral;
        private double previousError;
        private boolean initialized = false;

        private PIDF(double p, double i, double d, double f) {
            // fallback for if you don't supply initialI
            this(p, i, d, f, 0.0);
        }
        private PIDF(double p, double i, double d, double f, double initialI) {
            // initialize variables
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.integral = initialI;
            this.previousError = 0.0;
        }
        private double tick(double target, double current, double dt) {
            // tick one timestep
            double error = target - current;

            if (!initialized) {
                previousError = error;
                initialized = true;
            }

            // Integral term
            integral += error * dt;

            // Derivative term
            double derivative = dt > 0
                    ? (error - previousError) / dt
                    : 0.0;

            // Feedforward term
            double feedforward = f * target;

            // PIDF output
            double output = p * error +
                    i * integral +
                    d * derivative +
                    feedforward;

            // Store error for the next tick
            previousError = error;

            return output;
        }
    }

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
        double p = 0.005;
        double i = 0;
        double d = 0;
        double f = 0.0001667; // 0.46635714285
        flywheelPidf = new PIDF(p, i, d, f);

        // make turret angle PID
        p = 0;
        i = 0;
        d = 0;
        turretBasePid = new PIDF(p, i, d, 0);

        // make hive centers
        // TODO: add real values
        double leftHiveX = 30, leftHiveY = 0;
        double rightHiveX = -30, rightHiveY = 0;

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
        } else if (gamepad2.x) {
            rightTransferServo.setPower(OFF);
            leftTransferServo.setPower(OFF);
        } else if (gamepad2.dpad_down) {
            rightFlowerIntake.setPosition(armDownPos);
            leftFlowerIntake.setPosition(armDownPos);
        } else if (gamepad2.dpad_up) {
            rightFlowerIntake.setPosition(armUpPos);
            leftFlowerIntake.setPosition(armUpPos);
        }

        // tick PIDF
        closest = getClosestPoint(pose.position.x); // get closest hive position based on X position
        double goalY = closest.get(0);
        double goalX = closest.get(1);

        turretTargetAngle = Math.toDegrees(Math.atan2(pose.position.y - goalY, pose.position.x - goalX));
        turretTargetAngle = Math.max(lowerTurretLimit, Math.min(upperTurretLimit, turretTargetAngle));

        currentTime = System.nanoTime(); // the millisecond version is inaccurate for some reason, so we use nanoseconds
        double dt = (currentTime - lastTime) / 1_000_000; // nanoseconds -> milliseconds
        dt = Math.max(0.000001, dt); // avoid division by 0

        motorCurrentRPM = turretFlywheel.getVelocity() / MOTORTICKSPERORTATION * 60.0;
        telemetry.addData("current RPM", motorCurrentRPM);
        turretCurrentAngle = turretLastAngle + (rightFront.getVelocity() * (dt / 1_000.0) / SERVOTICKSPERORTATION) * 360.0;

        double servoPower = turretBasePid.tick(turretCurrentAngle, turretCurrentAngle, dt);
        servoPower = Math.max(OFF, Math.min(ON, servoPower));
//        turretBase.setPower(servoPower);

        if (flywheelPidfEnabled) {
            double motorPower = flywheelPidf.tick(targetRPM, motorCurrentRPM, dt);
            telemetry.addData("attempted power", motorPower);
            motorPower = Math.max(OFF, Math.min(ON, motorPower));
            turretFlywheel.setPower(motorPower);
            telemetry.addData("target RPM", targetRPM);
            telemetry.addData("current power", motorPower);
        } else {
            turretFlywheel.setPower(OFF);
        }

        lastTime = currentTime;
        telemetry.update();
    }
}
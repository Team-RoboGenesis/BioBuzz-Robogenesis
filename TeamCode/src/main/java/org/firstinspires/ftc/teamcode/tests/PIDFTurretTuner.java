package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "PIDF turret tuner")
public class PIDFTurretTuner extends OpMode {
    // config
    private double P = 0.05;
    private double I = 0;
    private double D = 0;
    private double initalI = 0;
    private final double SERVOTICKSPERORTATION = 4096;
    private final double lowerTurretLimit = 0;
    private final double upperTurretLimit = 360;

    // constants
    private final double OFF = 0;
    private final double ON = 1;

    // default values
    private double turretTargetAngle = 100;
    private double servoCurrentTicks = 0;
    private double turretCurrentAngle = 0;
    private double servoLastTicks = 0;
    private double turretLastAngle = 0;
    private double targetTicksPerMs = 1000;
    private double lastTime = System.nanoTime();
    private Map<Double, List<Double>> points = new HashMap<>();
    private double currentTime = 0;
    private MecanumDrive drive = null;
    private List<Double> closest = new ArrayList<>();

    // devices and PIDF
    private PIDF pid;
    private CRServo turretServo;
    private DcMotor rightFront = null;

    class PIDF {
        public double p, i, d, f;
        private double integral;
        private double previousError;
        private boolean initialized = false;

        private PIDF(double p, double i, double d) { this(p, i, d, 0.0); }
        private PIDF(double p, double i, double d, double initialI) {
            // initialize variables
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = 0;
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
        // PIDF setup
        pid = new PIDF(P, I, D, initalI);

        drive = new MecanumDrive(hardwareMap, new Pose2d(10, -53, Math.toRadians(90))); // TODO: get real values, revert x to 0

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        // device setup
        turretServo = hardwareMap.get(CRServo.class, "turretBase");

        // TODO: add real values
        double leftHiveX = 30, leftHiveY = 0;
        double rightHiveX = -30, rightHiveY = 0;

        points.put(leftHiveX, List.of(leftHiveX, leftHiveY));
        points.put(rightHiveX, List.of(rightHiveX, rightHiveY));

        // finish init
        telemetry.addLine("ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // tick PIDF
        currentTime = System.nanoTime(); // the millisecond version is inaccurate for some reason, so we use nanoseconds
        double dt = (currentTime - lastTime) / 1_000_000; // nanoseconds -> milliseconds
        dt = Math.max(0.000001, dt);

        servoCurrentTicks = rightFront.getCurrentPosition();
        Pose2d poseEstimate = drive.localizer.getPose();

        turretCurrentAngle = turretLastAngle + ((servoCurrentTicks - servoLastTicks) * SERVOTICKSPERORTATION / dt);

        closest = getClosestPoint(10); // get closest hive position based on X position TODO: replace with poseEstimate.position.x
        double goalX = 0; // TODO: REPLACE WITH closest.get(0)
        double goalY = 30; // TODO: REPLACE WITH closest.get(1)

        turretTargetAngle = Math.toDegrees(Math.atan2(40 - goalY, 40 - goalX)); // TODO: replace with poseEstimate.position.x
        turretTargetAngle = Math.max(lowerTurretLimit, Math.min(upperTurretLimit, turretTargetAngle));
        telemetry.addData("target turret angle", turretTargetAngle);
        telemetry.addData("current turret angle", turretCurrentAngle);

        double output = pid.tick(turretTargetAngle, turretCurrentAngle, dt);
        output = Math.max(OFF, Math.min(ON, output));
        turretServo.setPower(output);

        telemetry.addData("power", output);
        telemetry.update();


        lastTime = currentTime;
        turretLastAngle = turretCurrentAngle;
        servoLastTicks = servoCurrentTicks;
    }
}

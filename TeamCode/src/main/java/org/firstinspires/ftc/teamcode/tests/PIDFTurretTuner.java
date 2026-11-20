package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@TeleOp(name = "PIDF turret tuner")
public class PIDFTurretTuner extends OpMode {
    // config
    private double P = 0.005;
    private double I = 0;
    private double D = 0;
    private double initalI = 0;
    private final double TICKS_PER_TURRET_ROTATION = 1436;
    private final double lowerTurretLimit = -50;
    private final double upperTurretLimit = 124;

    // constants
    private final double OFF = 0;
    private final double ON = 1;

    // default values
    private double turretTargetAngle = 0;
    private double turretCurrentAngle = 0;
    private double turretLastAngle = 0;
    private double currentSecond = 0;
    private double startingTime = (double) System.nanoTime() / 1_000_000_000;
    private boolean dataCollection = true;
    private List<Double> currentAngles = new ArrayList<>();
    private List<Double> servoPowers = new ArrayList<>();
    private List<Double> voltages = new ArrayList<>();
    private List<Double> targetAngles = new ArrayList<>();
    private List<Double> times = new ArrayList<>();
    private List<List<Double>> data = new ArrayList<>();
    private double lastTime = System.nanoTime();
    private Map<Double, List<Double>> points = new HashMap<>();
    private double currentTime = 0;
    private int startTicks = 0;
    private MecanumDrive drive = null;
    private List<Double> closest = new ArrayList<>();

    // devices and PIDF
    private PIDF pid;
    private CRServo turretServo;
    private DcMotorEx rightFront = null;
    VoltageSensor voltageSensor;

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

        fileUtil.init(hardwareMap.appContext, "turret log");

        drive = new MecanumDrive(hardwareMap, new Pose2d(10, -53, Math.toRadians(90))); // TODO: get real values, revert x to 0

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        startTicks = rightFront.getCurrentPosition();

        data.add(times);
        data.add(currentAngles);
        data.add(servoPowers);
        data.add(voltages);
        data.add(targetAngles);

        // device setup
        turretServo = hardwareMap.get(CRServo.class, "turretBase");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

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
        double dt = (currentTime - lastTime) / 1_000_000_000; // nanoseconds -> seconds
        dt = Math.max(0.000001, dt);

        Pose2d poseEstimate = drive.localizer.getPose();

        int currentTicks = rightFront.getCurrentPosition() - startTicks;
        turretCurrentAngle = (currentTicks / TICKS_PER_TURRET_ROTATION) * 360.0;

        closest = getClosestPoint(10); // get closest hive position based on X position TODO: replace with poseEstimate.position.x
        double goalX = 0; // TODO: REPLACE WITH closest.get(0)
        double goalY = 30; // TODO: REPLACE WITH closest.get(1)

//        turretTargetAngle = Math.toDegrees(Math.atan2(40 - goalY, 40 - goalX)); // TODO: replace with poseEstimate.position.x
        if (gamepad1.a) {
            turretTargetAngle = 90;
        } else if (gamepad1.b) {
            turretTargetAngle = 0;
        } else if (gamepad1.y) {
            turretTargetAngle = 180;
        } else if (gamepad1.options && dataCollection) {
            dataCollection = false;
            String dataString = "";
            for (List<Double> point : data) {
                dataString += "\n" + point.stream()
                        .map(String::valueOf)
                        .collect(Collectors.joining(","));
            }
            fileUtil.overwriteFromIndex(dataString, 0);
            telemetry.addLine("loaded file");
            telemetry.addData("Current Angle Samples", currentAngles.size());
            telemetry.addData("Servo Power Samples", servoPowers.size());
            telemetry.addData("Voltage Samples", voltages.size());
            telemetry.addData("Target Samples", targetAngles.size());
            telemetry.addData("Export Length", dataString.length());
            telemetry.update();
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        turretTargetAngle = Math.max(lowerTurretLimit, Math.min(upperTurretLimit, turretTargetAngle));
        telemetry.addData("current turret ticks", rightFront.getCurrentPosition());
        telemetry.addData("target turret angle", turretTargetAngle);
        telemetry.addData("current turret angle", turretCurrentAngle);

        double servoPower = pid.tick(turretTargetAngle, turretCurrentAngle, dt);
        if (Math.abs(servoPower) < 0.25 && !(Math.abs(servoPower) < 0.01)) {
            servoPower = 0.25 * Math.signum(servoPower);
        }
        servoPower = Math.max(-ON, Math.min(ON, servoPower));
        turretServo.setPower(servoPower);

        if (dataCollection) {
            currentAngles.add(turretCurrentAngle);
            servoPowers.add(servoPower);
            voltages.add(voltageSensor.getVoltage());
            targetAngles.add(turretTargetAngle);
            currentSecond = currentTime / 1_000_000_000 - startingTime;
            times.add(currentSecond);
            telemetry.addData("current second", currentSecond);
        }

        telemetry.addData("power", servoPower);
        telemetry.update();


        lastTime = currentTime;
        turretLastAngle = turretCurrentAngle;
    }
}

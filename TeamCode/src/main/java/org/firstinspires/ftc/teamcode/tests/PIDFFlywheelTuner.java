package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@TeleOp(name = "PIDF flywheel tuner")
public class PIDFFlywheelTuner extends OpMode {
    // config
    private double P = 0.0005;
    private double I = 0;
    private double D = 0;
    private double F = 0.46635714285;
    private double initalI = 0;
    private final double MOTORTICKSPERORTATION = 28;

    // constants
    private final double OFF = 0;
    private final double ON = 1;

    // default values
    private double targetRPM = 0;
    private double motorCurrentTicks = 0;
    private double motorCurrentRPM = 0;
    private double lastTime = System.nanoTime();
    private double startingTime = System.nanoTime() / 1_000_000_000;
    private double currentTime = System.nanoTime();
    private List<Double> RPMs = new ArrayList<>();
    private List<Double> times = new ArrayList<>();
    private boolean dataCollection = false;
    private double currentSecond = 0;

    // devices and PIDF
    private PIDF pidf;
    private DcMotorEx turretFlywheel;

    class PIDF {
        public double p, i, d, f;
        private double integral;
        private double previousError;
        private boolean initialized = false;

        private PIDF(double p, double i, double d, double f) {
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

    @Override
    public void init() {
        // PIDF setup
        pidf = new PIDF(P, I, D, F, initalI);

        // device setup
        turretFlywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        turretFlywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretFlywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        fileUtil.init(hardwareMap.appContext, "log");

        // finish init
        telemetry.addLine("ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            targetRPM = 3000;
            dataCollection = true;
        } else if (gamepad1.b) {
            targetRPM = 0;
            dataCollection = false;
        } else if (gamepad1.right_bumper) {
            targetRPM = 800;
        } else if (gamepad1.dpad_up && dataCollection) {
            dataCollection = false;
            String RPMString = RPMs.stream()
                    .map(String::valueOf)
                    .collect(Collectors.joining(","));
            String timeString = times.stream()
                    .map(String::valueOf)
                    .collect(Collectors.joining(","));
            fileUtil.overwriteFromIndex(RPMString + "\n" + timeString, 0);
            telemetry.addLine("loaded file");
            telemetry.update();
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        telemetry.addData("target RPM", targetRPM);

        // tick PIDF
        currentTime = System.nanoTime(); // the millisecond version is inaccurate for some reason, so we use nanoseconds
        double dt = (currentTime - lastTime) / 1_000_000; // nanoseconds -> milliseconds
        dt = Math.max(0.000001, dt);

        motorCurrentRPM = turretFlywheel.getVelocity() / MOTORTICKSPERORTATION * 60.0;

        telemetry.addData("current RPM", motorCurrentRPM);

        double motorPower = pidf.tick(targetRPM, motorCurrentRPM, dt);
        motorPower = Math.max(OFF, Math.min(ON, motorPower));

        telemetry.addData("power", motorPower);
        turretFlywheel.setPower(motorPower);

        if (dataCollection) {
            RPMs.add(motorCurrentRPM);
            currentSecond = currentTime / 1_000_000_000 - startingTime;
            times.add(currentSecond);
            telemetry.addData("current second", currentSecond);
        }

        lastTime = currentTime;
        telemetry.update();
    }
}

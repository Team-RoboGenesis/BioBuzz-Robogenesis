package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@TeleOp(name = "PIDF flywheel tuner")
public class PIDFFlywheelTuner extends OpMode {
    // config
    private double P = 1.246;
    private double I = 0.0846;
    private double D = 0;
    private double F = 5.46;
    private double initalI = 0;
    private final double MOTORTICKSPERORTATION = 28;

    // constants
    private final double OFF = 0;
    private final double ON = 1;

    // default values
    private double targetRPM = 0;
    private double motorCurrentTicks = 0;
    private double motorCurrentTicksPerMs = 0;
    private double motorLastTicks = 0;
    private double targetTicksPerMs = 1000;
    private double lastTime = System.nanoTime();
    private double startingTime = System.nanoTime() / 1_000_000;
    private double currentTime = 0;
    private List<Double> RPMs = new ArrayList<>();
    private List<Double> times = new ArrayList<>();
    private boolean dataCollection = true;

    // devices and PIDF
    private PIDF pidf;
    private DcMotor turretFlywheel;

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
        turretFlywheel = hardwareMap.get(DcMotor.class, "flywheel");
        turretFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        } else if (gamepad1.b) {
            targetRPM = 0;
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
        }
        telemetry.addData("target RPM: ", targetRPM);
        // tick PIDF
        targetTicksPerMs = (targetRPM) / 60_000 * MOTORTICKSPERORTATION;

        currentTime = System.nanoTime(); // the millisecond version is inaccurate for some reason, so we use nanoseconds
        double dt = (currentTime - lastTime) / 1_000_000; // nanoseconds -> milliseconds
        dt = Math.max(0.000001, dt);

        motorCurrentTicks = turretFlywheel.getCurrentPosition();

        motorCurrentTicksPerMs = (motorCurrentTicks - motorLastTicks) / dt;

        double RPM = Math.floor(motorCurrentTicksPerMs / MOTORTICKSPERORTATION * 60_000);
        telemetry.addData("current RPM: ", RPM);

        double output = pidf.tick(targetTicksPerMs, motorCurrentTicksPerMs, dt);
        output = Math.max(-ON, Math.min(ON, output));
        telemetry.addData("power: ", output);
        turretFlywheel.setPower(output);

        if (dataCollection) {
            RPMs.add(RPM);
            times.add(currentTime / 1_000_000 - startingTime);
        }

        lastTime = currentTime;
        motorLastTicks = motorCurrentTicks;
        telemetry.update();
    }
}

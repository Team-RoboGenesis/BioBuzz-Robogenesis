package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PIDF test")
public class PIDFFlywheelTuner extends OpMode {
    // config
    private double P = 1;
    private double I = 1;
    private double D = 1;
    private double F = 1;
    private double initalI = 0;
    private final double MOTORTICKSPERORTATION = 28;

    // constants
    private final double OFF = 0;
    private final double ON = 1;

    // default values
    private double targetRPM = 100;
    private double motorCurrentTicks = 0;
    private double motorCurrentTicksPerMs = 0;
    private double motorLastTicks = 0;
    private double targetTicksPerMs = 1000;
    private double lastTime = 0;
    private double currentTime = 0;

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

        // finish init
        telemetry.addLine("ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // tick PIDF
        targetTicksPerMs = (targetRPM) / 60_000 * MOTORTICKSPERORTATION;

        currentTime = System.nanoTime(); // the millisecond version is inaccurate for some reason, so we use nanoseconds
        double dt = (currentTime - lastTime) / 1_000_000; // nanoseconds -> milliseconds
        dt = Math.max(0.000001, dt);

        motorCurrentTicks = turretFlywheel.getCurrentPosition();

        motorCurrentTicksPerMs = (motorCurrentTicks - motorLastTicks) / dt;

        double output = pidf.tick(targetTicksPerMs, motorCurrentTicksPerMs, dt);
        output = Math.max(OFF, Math.min(ON, output));
        turretFlywheel.setPower(output);


        lastTime = currentTime;
        motorLastTicks = motorCurrentTicks;
    }
}

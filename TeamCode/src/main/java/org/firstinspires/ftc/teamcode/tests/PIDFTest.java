package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PIDF test")
public class PIDFTest extends OpMode {
    // config
    private double P = 1;
    private double I = 1;
    private double D = 1;
    private double F = 1;
    private double initalI = 0;

    // default values
    private double targetTicksPerMs = 1000;
    private double lastTime = 0;
    private double currentTime = 0;
    private double currentTicksPerMs = 0;
    private double lastTicks = 0;
    private double currentTicks = 0;

    // devices and PIDF
    private PIDF pidf;
    private DcMotor flyWheel;

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
        flyWheel = hardwareMap.get(DcMotor.class, "flywheel");
        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // finish init
        telemetry.addLine("ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        currentTime = System.nanoTime();
        double dt = (currentTime - lastTime);
        dt = Math.max(0.001, dt);
        double output = pidf.tick(targetTicksPerMs, currentTicksPerMs, dt);
        output = Math.max(0.0, Math.min(1.0, output));
        flyWheel.setPower(output);

        lastTime = currentTime;
    }
}

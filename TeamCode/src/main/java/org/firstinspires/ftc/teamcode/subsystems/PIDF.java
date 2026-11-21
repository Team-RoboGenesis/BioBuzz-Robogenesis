package org.firstinspires.ftc.teamcode.subsystems;

public class PIDF {
    public double p, i, d, f;
    private double integral;
    private double previousError;
    private boolean initialized = false;

    public PIDF(double p, double i, double d, double f) {
        // fallback for if you don't supply initialI
        this(p, i, d, f, 0.0);
    }
    public PIDF(double p, double i, double d, double f, double initialI) {
        // initialize variables
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.integral = initialI;
        this.previousError = 0.0;
    }
    public double tick(double target, double current, double dt) {
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

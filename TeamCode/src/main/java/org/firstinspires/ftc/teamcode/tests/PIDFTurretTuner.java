package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PIDF test")
public class PIDFTurretTuner extends OpMode {
    // config
    private double P = 1;
    private double I = 1;
    private double D = 1;
    private double initalI = 0;
    private final double SERVOTICKSPERORTATION = 4096;

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
    private double currentTime = 0;

    // devices and PIDF
    private PIDF pid;
    private DcMotor turretServo;

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

    @Override
    public void init() {
        // PIDF setup
        pid = new PIDF(P, I, D, initalI);

        // device setup
        turretServo = hardwareMap.get(DcMotor.class, "flywheel");
        turretServo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        servoCurrentTicks = turretServo.getCurrentPosition(); // TODO: get servo-tied encoder here (will be a motor encoder)

        turretCurrentAngle = turretLastAngle + ((servoCurrentTicks - servoLastTicks) * SERVOTICKSPERORTATION / dt);

        double output = pid.tick(turretTargetAngle, turretCurrentAngle, dt);
        output = Math.max(OFF, Math.min(ON, output));
        turretServo.setPower(output);


        lastTime = currentTime;
        turretLastAngle = turretCurrentAngle;
        servoLastTicks = servoCurrentTicks;
    }
}

package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PIDF test")
public class PIDFTest extends OpMode {
    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double F = 0;
    private double integralSum = 0;
    private double error = 0;
    private double last_error = 0;
    private double lastError = 0;
    private DcMotor flyWheel;
    private double targetTicksPerMs = 1000;
    private double lastTime = 0;
    private double currentTime = 0;
    private double currentTicksPerMs = 0;
    private void sleep(double milis) {
        try {
            Thread.sleep((long) milis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void init() {
        flyWheel = hardwareMap.get(DcMotor.class, "flywheel");
        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("ready to start!");
        telemetry.update();
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime);
        if(dt <= 0){
            dt = 0.001;
        }
        currentTicksPerMs = flyWheel.getCurrentPosition() / dt;
        error = currentTicksPerMs - targetTicksPerMs;
        double pOut = P * error;
        integralSum += error * dt;
        double iOut = I * integralSum;
        double derivative = (error - last_error) / dt;
        double dOut = D * derivative;
        double fOut = F * targetTicksPerMs;
        double totalOutput = pOut + iOut + dOut + fOut;

        lastTime = System.currentTimeMillis();
        lastError = error;
    }
}

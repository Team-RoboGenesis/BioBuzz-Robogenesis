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
    private DcMotor flyWheel;
    private double targetTicksPerSec = 1000;
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
    }

    @Override
    public void loop() {

    }
}

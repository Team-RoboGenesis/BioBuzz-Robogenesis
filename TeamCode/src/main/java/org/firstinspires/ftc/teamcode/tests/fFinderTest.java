package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="f finder")
public class fFinderTest extends OpMode {
    private double power = 0;
    private DcMotorEx turretFlywheel;
    @Override
    public void init() {
        turretFlywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        turretFlywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretFlywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            power = 1;
        } else if (gamepad1.b) {
            power = 0;
        }

        turretFlywheel.setPower(power);

        power -= gamepad1.right_stick_y / 100;

        double velocity = turretFlywheel.getVelocity(); // Returns ticks per second
        double RPM = velocity / 28 * 60;
        telemetry.addData("Velocity (ticks/sec)", velocity);
        telemetry.addData("RPM", RPM);
        telemetry.addData("power", power);
        telemetry.update();
    }
}

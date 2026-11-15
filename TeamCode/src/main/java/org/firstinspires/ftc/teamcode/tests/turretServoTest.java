package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="turret servo test")
public class turretServoTest extends OpMode {
    private CRServo turretServo;
    private DcMotor rightFront = null;
    @Override
    public void init() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        turretServo = hardwareMap.get(CRServo.class, "turretBase");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            turretServo.setPower(1);
        } else if (gamepad1.b) {
            turretServo.setPower(0);
        } else if (gamepad1.y) {
            turretServo.setPower(-1);
        }
    }
}

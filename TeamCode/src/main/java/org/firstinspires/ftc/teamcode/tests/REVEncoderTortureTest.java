package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "REV encoder torture test")
public class REVEncoderTortureTest extends OpMode {
    private DcMotor throughBoreEncoder;
    private DcMotor throughBoreEncoderTheSecond;

    private DcMotor throughBoreEncoderTheThird;
    private DcMotor throughBoreEncoderTheFourth;


    private static final double TICKS_PER_REV = 8192.0;

    @Override
    public void init() {
        throughBoreEncoder = hardwareMap.get(DcMotor.class, "rightFront");
        throughBoreEncoderTheSecond = hardwareMap.get(DcMotor.class, "leftBack");
        throughBoreEncoderTheThird = hardwareMap.get(DcMotor.class, "leftFront");
        throughBoreEncoderTheFourth = hardwareMap.get(DcMotor.class, "rightBack");
        throughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throughBoreEncoderTheSecond.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throughBoreEncoderTheThird.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throughBoreEncoderTheFourth.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        throughBoreEncoderTheSecond.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        throughBoreEncoderTheThird.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        throughBoreEncoderTheFourth.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Initialized, Ready to start.");
        telemetry.update();

    }

    @Override
    public void loop() {
        int currentTicks = throughBoreEncoder.getCurrentPosition();
        int currentTicksTwo = throughBoreEncoderTheSecond.getCurrentPosition();
        int currentTicksThree = throughBoreEncoderTheThird.getCurrentPosition();
        int currentTicksFour = throughBoreEncoderTheFourth.getCurrentPosition();
        double rotations = currentTicks / TICKS_PER_REV;
        double degrees = rotations * 360.0;
        telemetry.addData("Encoder 1 ticks", currentTicks);
        telemetry.addData("Encoder 2 ticks", currentTicksTwo);
        telemetry.addData("Encoder 3 ticks", currentTicksThree);
        telemetry.addData("Encoder 4 ticks", currentTicksFour);
        telemetry.update();
    }
}

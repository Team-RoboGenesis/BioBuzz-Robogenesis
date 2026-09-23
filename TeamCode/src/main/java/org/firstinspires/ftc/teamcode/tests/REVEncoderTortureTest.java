package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "REV encoder torture test")
public class REVEncoderTortureTest extends OpMode {
    private DcMotor throughBoreEncoder;
    private DcMotor throughBoreEncoderTheSecond;

    private DcMotor throughBoreEncoderTheThird;

   

    private static final double TICKS_PER_REV = 8192.0;
    private Object currentTicksThree;

    @Override
    public void init() {
        throughBoreEncoder = hardwareMap.get(DcMotor.class, "encoderOne");
        throughBoreEncoderTheSecond = hardwareMap.get(DcMotor.class, "encoderTwo");
        throughBoreEncoderTheThird = hardwareMap.get(DcMotor.class, "encoderThree");
        throughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throughBoreEncoderTheSecond.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throughBoreEncoderTheThird.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        throughBoreEncoderTheSecond.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        throughBoreEncoderTheThird.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Initialized, Ready to start.");
        telemetry.update();

    }

    @Override
    public void loop() {

        int currentTicks = throughBoreEncoder.getCurrentPosition();
        int currentTicksTwo = throughBoreEncoderTheSecond.getCurrentPosition();
        int currentTicksThree = throughBoreEncoderTheThird.getCurrentPosition();
        double rotations = currentTicks / TICKS_PER_REV;
        double degrees = rotations * 360.0;
        telemetry.addData("Encoder 1 ticks", currentTicks);
        telemetry.addData("Encoder 2 ticks", currentTicksTwo);
        telemetry.addData("Encoder 3 ticks", currentTicksThree);
        telemetry.update();

    }
}

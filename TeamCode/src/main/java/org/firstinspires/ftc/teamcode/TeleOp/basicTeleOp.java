package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

class GateSensor {
    private DistanceSensor distanceSensor;
    private double threshold;
    private boolean last;
    private boolean entrance;

    public void init(String deviceName, double threshold) {
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, deviceName);
        this.threshold = threshold;
        this.last = false;
        this.entrance = deviceName.toLowerCase().contains("in"); // kinda dumb but it removes an argument
    }

    public int getData() {
        double distance = this.distanceSensor.getDistance(DistanceUnit.CM);
        boolean current = distance < this.threshold;

        if (current != this.last) {
            this.last = current;

            if (this.entrance) {
                return 1;
            } else {
                return -1;
            }
        }

        return 0;
    }
}

@TeleOp(name = "basic TeleOp")
public class basicTeleOp extends OpMode {
    // config
    private double nectarInThreshold = 10;
    private double pollenInThreshold = 10;
    private double nectarOutThreshold = 10;
    private double pollenOutThreshold = 10;

    // device setup
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private GateSensor nectarIn;
    private GateSensor nectarOut;
    private GateSensor pollenIn;
    private GateSensor pollenOut;
    private List<Servo> leds;

    // default values
    private int elementCount = 0;
    private boolean lastNectarIn = false;
    private boolean lastPollenIn = false;
    private boolean lastNectarOut = false;
    private boolean lastPollenOut = false;

    @Override
    public void init() {
        // make motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // make sensors
        nectarIn = new GateSensor();
        nectarOut = new GateSensor();
        pollenIn = new GateSensor();
        pollenOut = new GateSensor();
        nectarIn.init("nectarIn", nectarInThreshold);
        nectarOut.init("nectarOut", nectarInThreshold);
        pollenIn.init("pollenIn", pollenInThreshold);
        pollenOut.init("pollenOut", pollenOutThreshold);

        // make LED's
        leds.add(hardwareMap.get(Servo.class, "led1"));
        leds.add(hardwareMap.get(Servo.class, "led2"));
        leds.add(hardwareMap.get(Servo.class, "led3"));
        leds.add(hardwareMap.get(Servo.class, "led4"));
    }

    @Override
    public void loop() {
        // basic drivetrain setup
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        leftFront.setPower(y + x + rx);
        leftBack.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightBack.setPower(y + x - rx);

        // element counting with funny class
        elementCount += nectarIn.getData();
        elementCount += nectarOut.getData();
        elementCount += pollenIn.getData();
        elementCount += pollenOut.getData();
        for (Servo led : leds) {
            led.setPosition(0);
        }
        for (int i = 0; i < elementCount; i++) {
            leds.get(i).setPosition(1);
        }
    }

}

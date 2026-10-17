package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

class GateSensor {
    private DistanceSensor distanceSensor;
    private double threshold;
    private boolean last;
    private boolean entrance;
    private double color;
    private NormalizedColorSensor colorSensor;

    public void init(HardwareMap hwMap, String deviceName, double threshold) {
        this.distanceSensor = hwMap.get(DistanceSensor.class, deviceName);
        this.colorSensor = hwMap.get(NormalizedColorSensor.class, deviceName);
        this.threshold = threshold;
        this.last = false;
        this.entrance = deviceName.toLowerCase().contains("in"); // kinda dumb but it removes an argument
    }

    public int getData() {
        double distance = this.distanceSensor.getDistance(DistanceUnit.CM);
        boolean current = distance < this.threshold;

        if (current != this.last) {
            this.last = current;

            if (this.entrance && current) {
                return 1;
            } else if (!current) {
                return -1;
            }
        }

        return 0;
    }

    public double getColor() {
        NormalizedRGBA colors = this.colorSensor.getNormalizedColors();

        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        double hue = hsvValues[0];
        return hue / 360;
    }
}

@TeleOp(name = "basic TeleOp")
public class basicTeleOp extends OpMode {
    // constants
    private final double OFF = 0;
    private final double RED = 0.278;
    private final double BLUE = 0.611;
    private final double YELLOW = 0.388;
    private final double ON = 1;

    // config
    private final double distInThreshold = 10;
    private final double pollenInThreshold = 10;
    private final double distOutThreshold = 10;
    private final double pollenOutThreshold = 10;
    private final double nectarColor = RED;

    // device setup
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private GateSensor distIn;
    private GateSensor distOut;
    private GateSensor pollenIn;
    private GateSensor pollenOut;
    private List<Servo> leds = new ArrayList<>();
    private List<Double> elements = new ArrayList<>();
    private List<GateSensor> distanceSensors = new ArrayList<>();

    // starting values
    private boolean last = false;

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
        distIn = new GateSensor();
        distOut = new GateSensor();
        distIn.init(hardwareMap, "dist in", distInThreshold);
        distOut.init(hardwareMap,"dist out", distOutThreshold);
        distanceSensors.add(distIn);
        distanceSensors.add(distOut);

        // make LED's
        leds.add(hardwareMap.get(Servo.class, "led1"));
        leds.add(hardwareMap.get(Servo.class, "led2"));
        leds.add(hardwareMap.get(Servo.class, "led3"));
        leds.add(hardwareMap.get(Servo.class, "led4"));
    }

    @Override
    public void loop() {
        // element counting with funny class
        for (GateSensor distSen : distanceSensors) {
            int data = distSen.getData();
            if (data == 1) {
                elements.add(distSen.getColor());
            } else if (data == -1) {
                elements.remove(distSen.getColor());
            }
        }

        // led color setting
        for (Servo led : leds) {
            led.setPosition(OFF);
        }

        int i = 0;
        for (double elementColor : elements) {
            leds.get(i).setPosition(elementColor);
            i++;
        }

        if (elements.size() == 4) {
            // TODO: stop intake motor
        } else if (elements.size() > 4) {
            // TODO: reverse intake/launch excess
        }

        // button mapping
        // basic drivetrain setup
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        leftFront.setPower(y + x + rx);
        leftBack.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightBack.setPower(y + x - rx);

        if (last != gamepad2.a) {
            last = !last;
            if (gamepad2.a) {
                // TODO: run intake motor here
            } else {
                // TODO: stop intake motor here
            }
        }

        if (gamepad2.b) {
            // TODO: align & launch
        }
    }

}

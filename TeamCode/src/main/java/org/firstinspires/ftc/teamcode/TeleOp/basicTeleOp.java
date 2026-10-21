package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private final double INTAKEPOWER = 0.8;

    // config
    private final double distInThreshold = 10;
    private final double distOutThreshold = 10;

    // device setup
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private DcMotor intake = null;
    private DcMotor turretFlywheel = null;
    private CRServo turretBase = null;
    private CRServo flowerOuttake1 = null;
    private CRServo flowerOuttake2 = null;
    private CRServo transferServo1 = null;
    private CRServo transferServo2 = null;
    private CRServo transferServo3 = null;
    private CRServo transferServo4 = null;
    private GateSensor distIn;
    private GateSensor distOut;
    private GateSensor pollenIn;
    private GateSensor pollenOut;
    private List<Servo> leds = new ArrayList<>();
    private List<Double> elements = new ArrayList<>();
    private List<GateSensor> distanceSensors = new ArrayList<>();

    // starting values
    private boolean lastAPress = false;

    @Override
    public void init() {
        // make motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turretFlywheel = hardwareMap.get(DcMotor.class, "turret");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // make servos
        turretBase = hardwareMap.get(CRServo.class, "turretBase");
        flowerOuttake1 = hardwareMap.get(CRServo.class, "flowerOuttake1");
        flowerOuttake2 = hardwareMap.get(CRServo.class, "flowerOuttake2");
        transferServo1 = hardwareMap.get(CRServo.class, "transferServo1");
        transferServo2 = hardwareMap.get(CRServo.class, "transferServo2");
        transferServo3 = hardwareMap.get(CRServo.class, "transferServo3");
        transferServo4 = hardwareMap.get(CRServo.class, "transferServo4");

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
                if (!elements.isEmpty()) {
                    elements.remove(0);
                }
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

        // element check
        if (elements.size() == 4) {
            intake.setPower(OFF);
        } else if (elements.size() > 4) {
            // TODO: launch excess
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

        // on press toggle power levels
        if (lastAPress != gamepad2.a) {
            if (gamepad2.a) {
                if (intake.getPower() == 0) {
                    intake.setPower(INTAKEPOWER);
                } else {
                    intake.setPower(OFF);
                }
            }
            lastAPress = gamepad2.a;
        }

        // align & launch
        if (gamepad2.b) {
            // TODO: align & launch
        }

        // TODO: add PIDF regulation for turretFlywheel
    }
}
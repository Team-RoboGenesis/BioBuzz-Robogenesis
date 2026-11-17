package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@TeleOp(name="intake test")
public class intakeTest extends OpMode {
    private DcMotor intake = null;
    private Servo rightFlowerIntake = null;
    private Servo leftFlowerIntake = null;
    private CRServo rightTransferServo = null;
    private CRServo leftTransferServo = null;
    // config
    private double P = 0.0005;
    private double I = 0;
    private double D = 0;
    private double F = 0.434;
    private double initalI = 0;
    private final double MOTORTICKSPERORTATION = 28;

    // constants
    private final double OFF = 0;
    private final double ON = 1;

    // default values
    private double targetRPM = 0;
    private double motorCurrentRPM = 0;
    private double lastTime = System.nanoTime();
    private double startingTime = System.nanoTime() / 1_000_000_000;
    private double currentTime = System.nanoTime() + 10_000;
    private List<Double> RPMs = new ArrayList<>();
    private List<Double> motorPowers = new ArrayList<>();
    private List<Double> voltages = new ArrayList<>();
    private List<Double> targetRPMs = new ArrayList<>();
    private List<Double> times = new ArrayList<>();
    private List<List<Double>> data = new ArrayList<>();
    private boolean dataCollection = true;
    private double currentSecond = 0;
    private double right = 0;
    private double left = 0;

    // devices and PIDF
    private PIDF pidf;
    private DcMotorEx turretFlywheel;
    VoltageSensor voltageSensor;

    class PIDF {
        public double p, i, d, f;
        private double integral;
        private double previousError;
        private boolean initialized = false;

        private PIDF(double p, double i, double d, double f) {
            this(p, i, d, f, 0.0);
        }
        private PIDF(double p, double i, double d, double f, double initialI) {
            // initialize variables
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
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
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFlowerIntake = hardwareMap.get(Servo.class, "rightFlowerIntake");
        leftFlowerIntake = hardwareMap.get(Servo.class, "leftFlowerIntake");
        rightTransferServo = hardwareMap.get(CRServo.class, "rightTransferServo");
        leftTransferServo = hardwareMap.get(CRServo.class, "leftTransferServo");

        // PIDF setup
        pidf = new PIDF(P, I, D, F, initalI);

        // device setup
        turretFlywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        turretFlywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretFlywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        fileUtil.init(hardwareMap.appContext, "p 0.0005");

        // data collection list setup
        data.add(times);
        data.add(targetRPMs);
        data.add(RPMs);
        data.add(motorPowers);
        data.add(voltages);



        // finish init
        telemetry.addLine("ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intake.setPower(1);
        } else if (gamepad1.b) {
            intake.setPower(0);
        } else if (gamepad1.y) {
            rightTransferServo.setPower(1);
            leftTransferServo.setPower(-1);
        } else if (gamepad1.x) {
            rightTransferServo.setPower(0);
            leftTransferServo.setPower(0);
        } else if (gamepad1.right_bumper) {
            targetRPM = 3000;
        } else if (gamepad1.left_bumper) {
            targetRPM = 0;
        } else if (gamepad1.right_stick_button) {
            targetRPM = 4000;
        } else if (gamepad1.options && dataCollection) {
            dataCollection = false;
            String dataString = "";
            for (List<Double> point : data) {
                dataString += "\n" + point.stream()
                        .map(String::valueOf)
                        .collect(Collectors.joining(","));
            }
            fileUtil.overwriteFromIndex(dataString, 0);
            telemetry.addLine("loaded file");
            telemetry.update();
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        targetRPM -= gamepad1.left_stick_y;
        right -= gamepad1.right_stick_y / 500;
        left += gamepad1.right_stick_x / 500;

        telemetry.addData("target RPM: ", targetRPM);

        rightFlowerIntake.setPosition(right);
        leftFlowerIntake.setPosition(left);

        // tick PIDF
        currentTime = System.nanoTime(); // the millisecond version is inaccurate for some reason, so we use nanoseconds
        double dt = (currentTime - lastTime) / 1_000_000; // nanoseconds -> milliseconds
        dt = Math.max(0.000001, dt);

        motorCurrentRPM = turretFlywheel.getVelocity() / MOTORTICKSPERORTATION * 60.0;

        telemetry.addData("current RPM", motorCurrentRPM);

        double motorPower = pidf.tick(targetRPM, motorCurrentRPM, dt);
        motorPower = Math.max(OFF, Math.min(ON, motorPower));

        telemetry.addData("power", motorPower);
        turretFlywheel.setPower(motorPower);

        if (dataCollection) {
            RPMs.add(motorCurrentRPM);
            motorPowers.add(motorPower);
            voltages.add(voltageSensor.getVoltage());
            targetRPMs.add(targetRPM);
            currentSecond = currentTime / 1_000_000_000 - startingTime;
            times.add(currentSecond);
            telemetry.addData("current second", currentSecond);
        }

        lastTime = currentTime;
        telemetry.update();
    }
}

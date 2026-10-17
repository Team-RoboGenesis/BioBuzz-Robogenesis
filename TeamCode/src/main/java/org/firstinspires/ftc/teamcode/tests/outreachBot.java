package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "outreach bot")
public class outreachBot extends OpMode {
    private List<Servo> leds = new ArrayList<>();
    private List<Servo> frozenLeds = new ArrayList<>();
    private List<Servo> ledsLeft = new ArrayList<>();
    private List<Double> colors = new ArrayList<>();
    private double speed = 0;
    private double difficulty = 0;
    private boolean last = false;

    private void sleep(double milis){
        try {
            Thread.sleep((long) milis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void init() {
        leds.add(hardwareMap.get(Servo.class, "led1"));
        leds.add(hardwareMap.get(Servo.class, "led2"));
        leds.add(hardwareMap.get(Servo.class, "led3"));
        leds.add(hardwareMap.get(Servo.class, "led4"));
    }

    @Override
    public void loop() {
        // get data
        if (gamepad2.touchpad_finger_1) {
            speed = (gamepad2.touchpad_finger_1_x + 1) / 2;
            difficulty = (gamepad2.touchpad_finger_1_y + 1) / 2;
        }

        // button pressing logic
        if (last != gamepad1.a) {
            last = gamepad1.a;
            if (gamepad1.a && !(frozenLeds.size() == leds.size())) {
                frozenLeds.add(leds.get(frozenLeds.size()));
            }
        }

        // update ledsLeft
        ledsLeft.clear();
        ledsLeft.addAll(leds);
        ledsLeft.removeAll(frozenLeds);

        // color LEDs
        for (Servo led : ledsLeft) {
            double power = led.getPosition() + speed / 5;
            if (power > 1) {
                power = 0;
            }
            led.setPosition(power);
        }

        if (frozenLeds.size() == leds.size()) {
            for (Servo led : leds) {
                colors.add(led.getPosition());
            }

            double totalDistance = 0.0;

            for (int i = 0; i < colors.size() - 1; i++) {
                totalDistance += Math.abs(colors.get(i + 1) - colors.get(i));
            }

            double averageDistance = totalDistance / (colors.size() - 1);

            sleep(5000);

            if (averageDistance <= difficulty) {
                for (Servo led : leds) {
                    led.setPosition(0.5);
                }
            } else {
                for (Servo led : leds) {
                    led.setPosition(0.278);
                }
            }

            while (!gamepad1.b) {
                sleep(100);
            }
            
            frozenLeds.clear();
        }
        sleep(10);
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class GateSensor {
    private DistanceSensor distanceSensor;
    private double threshold;
    private boolean last;
    private boolean entrance;
    private NormalizedColorSensor colorSensor;
    private boolean color;

    public GateSensor(HardwareMap hwMap, String deviceName, double threshold, boolean color) {
        this.distanceSensor = hwMap.get(DistanceSensor.class, deviceName);
        this.color = color;
        if (color) {
            this.colorSensor = hwMap.get(NormalizedColorSensor.class, deviceName);
        }
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
        if (!color) {
            return 0;
        }

        NormalizedRGBA colors = this.colorSensor.getNormalizedColors();
        // get colors with colors.red, green and blue

        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        double hue = hsvValues[0];
        return hue / 360;
    }
}

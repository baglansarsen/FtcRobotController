package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A mechanism class for a color sensor, specifically for artifact sorting.
 * Assumes hardware map name is "sensor_color_center".
 */
public class ColorSensorDetector {
    // Sensor declaration.
    private ColorSensor colorSensor = null;

    public ColorSensorDetector() {
        // Default constructor
    }

    /**
     * Initializes the color sensor.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize ColorSensor using the variable name as the hardware map name.
        colorSensor = hwMap.get(ColorSensor.class, "sensor_color_center");

        // Optional: Enable the LED light on the color sensor. Useful for consistent readings.
        // This line assumes the sensor has an LED and it's a good practice for your use case.
        colorSensor.enableLed(true);
    }

    /**
     * @return The red value reported by the color sensor.
     */
    public int getRed() {
        return colorSensor.red();
    }

    /**
     * @return The green value reported by the color sensor.
     */
    public int getGreen() {
        return colorSensor.green();
    }

    /**
     * @return The blue value reported by the color sensor.
     */
    public int getBlue() {
        return colorSensor.blue();
    }

    /**
     * @return The combined total light value (Alpha) reported by the color sensor.
     */
    public int getAlpha() {
        return colorSensor.alpha();
    }

    /**
     * Helper method to determine if the sensor is seeing a dominant red color.
     * NOTE: You will need to calibrate the threshold values (e.g., 50, 1.5) based on your environment.
     * @return True if the detected color is predominantly red.
     */
    public boolean isRedDominant() {
        int r = getRed();
        int g = getGreen();
        int b = getBlue();
        
        // Check if red value is above a certain minimum threshold and significantly higher than green and blue.
        return r > 50 && r > g * 1.5 && r > b * 1.5;
    }

    /**
     * Helper method to determine if the sensor is seeing a dominant blue color.
     * NOTE: You will need to calibrate the threshold values (e.g., 50, 1.5) based on your environment.
     * @return True if the detected color is predominantly blue.
     */
    public boolean isBlueDominant() {
        int r = getRed();
        int g = getGreen();
        int b = getBlue();
        
        // Check if blue value is above a certain minimum threshold and significantly higher than red and green.
        return b > 50 && b > r * 1.5 && b > g * 1.5;
    }
}
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A mechanism class for a color sensor that also provides distance sensing.
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
        // Initialize ColorSensor. This same device can be used as a DistanceSensor.
        colorSensor = hwMap.get(ColorSensor.class, "sensor_color_center");

        // Set the gain for the color sensor.
        if (colorSensor instanceof NormalizedColorSensor) {
            ((NormalizedColorSensor) colorSensor).setGain(10);
        }

        // Enable the LED light on the color sensor for consistent readings.
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
     * Gets the distance reading from the sensor.
     * @param unit The desired unit of distance (e.g., DistanceUnit.CM).
     * @return The distance to the nearest object.
     */
    public double getDistance(DistanceUnit unit) {
        // Cast the ColorSensor to a DistanceSensor to get the distance reading.
        return ((DistanceSensor) colorSensor).getDistance(unit);
    }

    /**
     * Helper method to determine if the sensor is seeing a dominant red color.
     * NOTE: You will need to calibrate the threshold values.
     * @return True if the detected color is predominantly red.
     */
    public boolean isRedDominant() {
        int r = getRed();
        int g = getGreen();
        int b = getBlue();
        return r > 50 && r > g * 1.5 && r > b * 1.5;
    }

    /**
     * Helper method to determine if the sensor is seeing a dominant blue color.
     * NOTE: You will need to calibrate the threshold values.
     * @return True if the detected color is predominantly blue.
     */
    public boolean isBlueDominant() {
        int r = getRed();
        int g = getGreen();
        int b = getBlue();
        return b > 50 && b > r * 1.5 && b > g * 1.5;
    }
}
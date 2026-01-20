package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A mechanism class for managing multiple color/distance sensors.
 * It now handles left, center, and right sensors.
 */
public class ColorSensorDetector {
    // Enum to specify which sensor to use
    public enum SensorLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    // Sensor declarations
    private ColorSensor sensor_color_left = null;
    private ColorSensor sensor_color_center = null;
    private ColorSensor sensor_color_right = null;

    // Distance threshold for an artifact being "staged" or "present"
    private static final double STAGING_DISTANCE_CM = 5.0; // Assume a closer distance for precise staging

    public ColorSensorDetector() {
        // Default constructor
    }

    /**
     * Initializes all three color sensors.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize all sensors from the hardware map
        sensor_color_left = hwMap.get(ColorSensor.class, "sensor_color_left");
        sensor_color_center = hwMap.get(ColorSensor.class, "sensor_color_center");
        sensor_color_right = hwMap.get(ColorSensor.class, "sensor_color_right");

        // Configure all sensors with the same settings
        configureSensor(sensor_color_left);
        configureSensor(sensor_color_center);
        configureSensor(sensor_color_right);
    }

    /**
     * Helper method to apply common settings to a sensor.
     * @param sensor The sensor to configure.
     */
    private void configureSensor(ColorSensor sensor) {
        if (sensor instanceof NormalizedColorSensor) {
            ((NormalizedColorSensor) sensor).setGain(10);
        }
        sensor.enableLed(true);
    }

    /**
     * Gets the specified sensor object based on its location.
     * @param location The location of the sensor to get (LEFT, CENTER, or RIGHT).
     * @return The ColorSensor object. Returns the center sensor by default.
     */
    private ColorSensor getSensor(SensorLocation location) {
        switch (location) {
            case LEFT:
                return sensor_color_left;
            case RIGHT:
                return sensor_color_right;
            case CENTER:
            default:
                return sensor_color_center;
        }
    }
    
    /**
     * Checks if an artifact is correctly positioned under the LEFT lift.
     * The condition is that the sensor sees the artifact (close enough).
     * @return True if the artifact is staged left.
     */
    public boolean isStagedLeft() {
        return getDistance(SensorLocation.LEFT, DistanceUnit.CM) < STAGING_DISTANCE_CM;
    }

    /**
     * Checks if an artifact is correctly positioned under the RIGHT lift.
     * The condition is that the sensor sees the artifact (close enough).
     * @return True if the artifact is staged right.
     */
    public boolean isStagedRight() {
        return getDistance(SensorLocation.RIGHT, DistanceUnit.CM) < STAGING_DISTANCE_CM;
    }

    /**
     * Checks if an artifact is correctly positioned under the CENTER sensor.
     * The condition is that the sensor sees the artifact (close enough).
     * @return True if the artifact is staged center.
     */
    public boolean isStagedCenter() {
        return getDistance(SensorLocation.CENTER, DistanceUnit.CM) < STAGING_DISTANCE_CM;
    }
    
    // --- Accessor methods ---

    public int getRed(SensorLocation location) { return getSensor(location).red(); }
    public int getGreen(SensorLocation location) { return getSensor(location).green(); }
    public int getBlue(SensorLocation location) { return getSensor(location).blue(); }

    public double getDistance(SensorLocation location, DistanceUnit unit) {
        return ((DistanceSensor) getSensor(location)).getDistance(unit);
    }
    
    // --- Legacy methods (Default to CENTER sensor) ---

    /** @deprecated Use getRed(SensorLocation.CENTER) instead. */
    @Deprecated
    public int getRed() { return getRed(SensorLocation.CENTER); }

    /** @deprecated Use getGreen(SensorLocation.CENTER) instead. */
    @Deprecated
    public int getGreen() { return getGreen(SensorLocation.CENTER); }

    /** @deprecated Use getBlue(SensorLocation.CENTER) instead. */
    @Deprecated
    public int getBlue() { return getBlue(SensorLocation.CENTER); }

    /** @deprecated Use getDistance(SensorLocation.CENTER, unit) instead. */
    @Deprecated
    public double getDistance(DistanceUnit unit) { return getDistance(SensorLocation.CENTER, unit); }
}
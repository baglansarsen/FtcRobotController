package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A mechanism class for a Distance Sensor.
 * Assumes hardware map name is "sensor_color_center_DistanceSensor".
 */
public class DistanceSensorDetector {
    // Sensor declaration.
    private DistanceSensor distanceSensor = null;

    public DistanceSensorDetector() {
        // Default constructor
    }

    /**
     * Initializes the distance sensor.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize DistanceSensor
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_center_DistanceSensor");
    }

    /**
     * Gets the distance reading from the sensor.
     * @param unit The desired unit of distance (e.g., DistanceUnit.CM).
     * @return The distance to the nearest object.
     */
    public double getDistance(DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }
}
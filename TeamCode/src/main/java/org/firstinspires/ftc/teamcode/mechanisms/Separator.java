package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A mechanism class for a Continuous Rotation Servo (CRServo) used to sort artifacts.
 * The assumed hardware map name is "seperater".
 */
public class Separator {

    public enum Direction {
        LEFT,
        RIGHT
    }

    // Servo declaration.
    private CRServo seperater = null;

    // Constant for sorting speed (can be adjusted)
    private static final double SORT_SPEED = 1.0;

    public Separator() {
        // Default constructor
    }

    /**
     * Initializes the separator servo.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize CRServo using the variable name as the hardware map name.
        // NOTE: The servo should be configured as a continuous rotation servo in the REV Control Hub configuration.
        seperater = hwMap.get(CRServo.class, "seperater");

        // Ensure the servo is stopped initially.
        seperater.setPower(0.0);
    }

    /**
     * Runs the servo to sort artifacts to the left.
     * Assuming negative speed moves to the left.
     */
    public void sortLeft() {
        seperater.setPower(-SORT_SPEED);
    }

    /**
     * Runs the servo to sort artifacts to the right.
     * Assuming positive speed moves to the right.
     */
    public void sortRight() {
        seperater.setPower(SORT_SPEED);
    }

    /**
     * Stops the sorting servo.
     */
    public void stop() {
        seperater.setPower(0.0);
    }

    /**
     * Runs the servo at a custom speed.
     * @param speed The speed (from -1.0 to 1.0) to run the servo.
     */
    public void setSpeed(double speed) {
        seperater.setPower(speed);
    }
}
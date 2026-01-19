package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A mechanism class for a single motor shooter/thrower system.
 * Assumes hardware map name is "right_throw".
 */
public class Shooter {
    // Motor declaration.
    private DcMotor right_throw = null;

    // Constant for shooter speed (can be adjusted)
    private static final double SHOOT_SPEED = 1.0;

    public Shooter() {
        // Default constructor
    }

    /**
     * Initializes the shooter motor.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize motor
        right_throw = hwMap.get(DcMotor.class, "right_throw");

        // Set motor direction based on user's configuration
        right_throw.setDirection(DcMotor.Direction.FORWARD);

        // Set motor mode based on user's configuration
        right_throw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor zero power behavior based on user's configuration
        right_throw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Sets the power for the shooter motor.
     * @param power The power (from 0.0 to 1.0) to run the motor.
     */
    public void setPower(double power) {
        // Ensure power is non-negative since shooters usually run in one direction
        right_throw.setPower(Math.abs(power));
    }

    /**
     * Runs the shooter motor at the default speed.
     */
    public void shoot() {
        setPower(SHOOT_SPEED);
    }

    /**
     * Stops the shooter motor.
     */
    public void stop() {
        setPower(0.0);
    }
}
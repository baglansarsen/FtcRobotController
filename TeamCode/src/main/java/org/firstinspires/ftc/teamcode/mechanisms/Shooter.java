package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * A mechanism class for a single motor shooter/thrower system, now using velocity control.
 * Assumes hardware map name is "right_throw".
 */
public class Shooter {
    // Motor declaration must be DcMotorEx for velocity methods.
    private DcMotorEx right_throw = null;
    private double currentTargetVelocity = 0.0; // Tracks the last velocity set via setVelocity()

    // Constants for velocity control (TUNE THESE BASED ON YOUR MOTOR/GEARING)
    public static final double TARGET_VELOCITY_COUNTS_PER_SEC = 1800; // Example target velocity
    private static final double VELOCITY_TOLERANCE = 100; // Tolerance for checking if speed is reached

    public Shooter() {
        // Default constructor
    }

    /**
     * Initializes the shooter motor for velocity control.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize motor as DcMotorEx
        right_throw = hwMap.get(DcMotorEx.class, "right_throw");

        // Set motor direction to FORWARD. 
        // If the encoder is wired backwards, we handle it in getVelocity/isAtTargetVelocity.
        right_throw.setDirection(DcMotor.Direction.FORWARD);

        // Set motor zero power behavior to FLOAT (confirmed by user)
        right_throw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set motor mode to RUN_USING_ENCODER for velocity control
        right_throw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional: Set PIDF coefficients if default values are insufficient (use defaults first)
        PIDFCoefficients defaultCoeffs = right_throw.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        right_throw.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
            defaultCoeffs.p, defaultCoeffs.i, defaultCoeffs.d, defaultCoeffs.f * 1.2
        ));
    }

    /**
     * Sets the target velocity for the shooter motor.
     * @param velocity The velocity in encoder counts per second.
     */
    public void setVelocity(double velocity) {
        this.currentTargetVelocity = velocity; // Store the target velocity locally
        right_throw.setVelocity(velocity);
    }

    /**
     * Sets the power for the shooter motor (Legacy method for simple testing).
     * @param power The power (from 0.0 to 1.0) to run the motor.
     */
    public void setPower(double power) {
        // Since we are primarily using velocity, this method now translates to setting velocity
        // based on the maximum velocity.
        right_throw.setPower(Math.abs(power));
    }

    /**
     * Checks if the shooter has reached its target velocity within the tolerance.
     * Uses absolute values to handle potential negative encoder readings when motor is spinning correctly.
     * @return True if shooter is at target speed.
     */
    public boolean isAtTargetVelocity() {
        double currentSpeed = Math.abs(right_throw.getVelocity());
        double targetSpeed = Math.abs(this.currentTargetVelocity);
        return Math.abs(currentSpeed - targetSpeed) < VELOCITY_TOLERANCE;
    }

    /**
     * Gets the current velocity of the shooter motor.
     * @return The absolute velocity (speed) in encoder counts per second.
     */
    public double getVelocity() {
        return Math.abs(right_throw.getVelocity());
    }

    /**
     * Stops the shooter motor.
     */
    public void stop() {
        this.currentTargetVelocity = 0.0; // Reset target on stop
        right_throw.setVelocity(0); // Stop by setting target velocity to zero
    }
}
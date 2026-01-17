package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A mechanism class for a single motor intake system.
 */
public class Intake {
    // Motor declaration. Assuming hardware map name is "intake_front".
    private DcMotor intake_front = null;

    public Intake() {
        // Default constructor
    }

    /**
     * Initializes the intake motor.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize motor using the variable name as the hardware map name.
        intake_front = hwMap.get(DcMotor.class, "intake_front");

        // Set motor direction: This might need adjustment based on the physical mechanism.
        // Assuming FORWARD direction pulls material IN.
        intake_front.setDirection(DcMotor.Direction.FORWARD);

        // Set motor zero power behavior to BRAKE
        intake_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor mode to RUN_WITHOUT_ENCODER for simple speed control
        intake_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the power for the intake motor to pull material IN.
     * @param power The power (from 0.0 to 1.0) to run the intake.
     */
    public void in(double power) {
        // Use the absolute value to ensure power is positive, and cap it at 1.0
        intake_front.setPower(Math.min(1.0, Math.abs(power)));
    }

    /**
     * Sets the power for the intake motor to push material OUT (eject).
     * @param power The power (from 0.0 to 1.0) to run the intake.
     */
    public void out(double power) {
        // Run in the reverse direction.
        intake_front.setPower(-Math.min(1.0, Math.abs(power)));
    }

    /**
     * Stops the intake motor by setting power to zero.
     */
    public void stop() {
        intake_front.setPower(0.0);
    }
}
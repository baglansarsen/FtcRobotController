package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * A mechanism class for a dual-motor lift system to raise artifacts to a shooter.
 * Assumes hardware map names are "right_lift" and "left_lift".
 */
public class Lift {
    // Motor declarations.
    private DcMotor right_lift = null;
    private DcMotor left_lift = null;

    // Constant for lift speed (can be adjusted)
    private static final double LIFT_SPEED = 1.0;

    public Lift() {
        // Default constructor
    }

    /**
     * Initializes the lift motors.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize motors
        right_lift = hwMap.get(DcMotor.class, "right_lift");
        left_lift = hwMap.get(DcMotor.class, "left_lift");

        // Set motor directions based on the user's latest configuration:
        right_lift.setDirection(DcMotor.Direction.REVERSE);
        left_lift.setDirection(DcMotor.Direction.REVERSE); 

        // Set motor zero power behavior to BRAKE (confirmed by user)
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor mode for simple speed control
        right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the same power for both lift motors.
     * @param power The power (from -1.0 to 1.0) to run the motors.
     */
    public void setPower(double power) {
        right_lift.setPower(power);
        left_lift.setPower(power);
    }

    /**
     * Sets the power for each lift motor individually.
     * @param rightPower The power for the right motor (-1.0 to 1.0).
     * @param leftPower The power for the left motor (-1.0 to 1.0).
     */
    public void setIndividualPower(double rightPower, double leftPower) {
        right_lift.setPower(rightPower);
        left_lift.setPower(leftPower);
    }

    /**
     * Runs the lift motors UP at the default speed.
     */
    public void up() {
        setPower(LIFT_SPEED);
    }

    /**
     * Runs the lift motors DOWN at the default speed.
     */
    public void down() {
        setPower(-LIFT_SPEED);
    }

    /**
     * Stops the lift motors.
     */
    public void stop() {
        setPower(0.0);
    }
}
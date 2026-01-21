package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * A mechanism class for a single motor shooter/thrower system, now using velocity control.
 * Assumes hardware map name is "right_throw".
 */
public class Shooter {
    private DcMotorEx right_throw = null;
    private double currentTargetVelocity = 0.0;

    public static final double TARGET_VELOCITY_COUNTS_PER_SEC = 1800;
    private static final double VELOCITY_TOLERANCE = 100;

    public Shooter() {}

    public void init(HardwareMap hwMap) {
        right_throw = hwMap.get(DcMotorEx.class, "right_throw");
        // With correct wiring, FORWARD should be the correct shooting direction.
        right_throw.setDirection(DcMotor.Direction.REVERSE);
        right_throw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_throw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients defaultCoeffs = right_throw.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        right_throw.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
            defaultCoeffs.p, defaultCoeffs.i, defaultCoeffs.d, defaultCoeffs.f * 1.2
        ));
    }

    public void setVelocity(double velocity) {
        this.currentTargetVelocity = velocity;
        right_throw.setVelocity(velocity);
    }

    /**
     * Checks if the shooter has reached its target velocity within the tolerance.
     * Now that wiring is fixed, we use the actual velocity without Math.abs().
     * @return True if shooter is at target speed.
     */
    public boolean isAtTargetVelocity() {
        double currentVelocity = right_throw.getVelocity();
        return Math.abs(currentVelocity - this.currentTargetVelocity) < VELOCITY_TOLERANCE;
    }

    /**
     * Gets the current velocity of the shooter motor.
     * Now that wiring is fixed, we return the actual velocity without Math.abs().
     * @return The velocity in encoder counts per second.
     */
    public double getVelocity() {
        return right_throw.getVelocity();
    }

    public void stop() {
        this.currentTargetVelocity = 0.0;
        right_throw.setVelocity(0);
    }
}
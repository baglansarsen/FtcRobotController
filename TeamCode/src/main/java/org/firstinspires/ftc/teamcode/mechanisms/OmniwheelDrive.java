package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A mechanism class for an omniwheel drivetrain (also known as Mecanum).
 * It includes motor initialization and drive methods.
 */
public class OmniwheelDrive {
    // Motor declarations based on user request.
    // Assuming hardware map names match these variable names:
    private DcMotor left_front = null;
    private DcMotor left_back = null;
    private DcMotor right_front = null;
    private DcMotor right_back = null;

    public OmniwheelDrive() {
        // Default constructor
    }

    /**
     * Initializes the drivetrain motors.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize motors using the variable names as the hardware map names.
        left_front  = hwMap.get(DcMotor.class, "left_front");
        left_back = hwMap.get(DcMotor.class, "left_back");
        right_front = hwMap.get(DcMotor.class, "right_front");
        right_back = hwMap.get(DcMotor.class, "right_back");

        // Set motor directions: This is a crucial step and needs to be correct for Mecanum drive.
        // Assuming a standard configuration where left side is reversed for forward motion.
        // **These directions may need adjustment based on the physical robot and wheel mounting!**
        left_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.FORWARD);

        // Set motor zero power behavior to BRAKE by default.
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor modes to RUN_WITHOUT_ENCODER for simple speed control in TeleOp
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set the zero power behavior for all motors.
     * @param behavior The desired ZeroPowerBehavior (e.g., BRAKE or FLOAT).
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        left_front.setZeroPowerBehavior(behavior);
        left_back.setZeroPowerBehavior(behavior);
        right_front.setZeroPowerBehavior(behavior);
        right_back.setZeroPowerBehavior(behavior);
    }

     /**
     * Set the run mode for all motors.
     * @param mode The desired DcMotor.RunMode (e.g., RUN_WITHOUT_ENCODER).
     */
    public void setRunMode(DcMotor.RunMode mode) {
        left_front.setMode(mode);
        left_back.setMode(mode);
        right_front.setMode(mode);
        right_back.setMode(mode);
    }


    /**
     * Implements omniwheel (Mecanum) drive movement.
     * @param forward Backward/Forward power (Y axis).
     * @param strafe Left/Right strafing power (X axis).
     * @param rotation Clockwise/Counter-clockwise rotation power.
     */
    public void drive(double forward, double strafe, double rotation) {
        // Mecanum drive calculations
        double frontLeftPower = forward + strafe + rotation;
        double frontRightPower = forward - strafe - rotation;
        double backLeftPower = forward - strafe + rotation;
        double backRightPower = forward + strafe - rotation;

        // Normalize the values so none exceed 1.0 (or -1.0)
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Send power to the motors
        left_front.setPower(frontLeftPower);
        right_front.setPower(frontRightPower);
        left_back.setPower(backLeftPower);
        right_back.setPower(backRightPower);
    }

    /**
     * Stop all motors by setting all drive power to zero.
     */
    public void stop() {
        drive(0, 0, 0);
    }
}
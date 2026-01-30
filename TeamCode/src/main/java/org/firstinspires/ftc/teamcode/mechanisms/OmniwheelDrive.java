package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A mechanism class for an omniwheel drivetrain (also known as Mecanum).
 * It includes motor initialization and drive methods, now supporting encoder movement for Autonomous.
 */
public class OmniwheelDrive {
    // Motor declarations based on user request.
    private DcMotor left_front = null;
    private DcMotor left_back = null;
    private DcMotor right_front = null;
    private DcMotor right_back = null;
    private IMU imu = null;

    // --- ENCODER CONSTANTS ---
    static final double COUNTS_PER_MOTOR_REV = 384.5;    // Gobilda 435 RPM motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No external gearing
    static final double WHEEL_DIAMETER_INCHES = 3.78;   // 96mm Mecanum wheels
    static final double COUNTS_PER_CHASSIS_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    public static final double COUNTS_PER_INCH = COUNTS_PER_CHASSIS_REV /
                                               (WHEEL_DIAMETER_INCHES * 3.141592653589793);
    public static final double DRIVE_SPEED = 0.5;


    public OmniwheelDrive() {
        // Default constructor
    }

    /**
     * Initializes the drivetrain motors and IMU.
     * @param hwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Initialize motors using the variable names as the hardware map names.
        left_front  = hwMap.get(DcMotor.class, "left_front");
        left_back = hwMap.get(DcMotor.class, "left_back");
        right_front = hwMap.get(DcMotor.class, "right_front");
        right_back = hwMap.get(DcMotor.class, "right_back");

        // Initialize the IMU assuming the hardware map name is "imu"
        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Set motor directions (STANDARD CONFIGURATION FOR MECANUM: Left REVERSE, Right FORWARD)
        left_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.REVERSE); 
        right_front.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.FORWARD);

        // Set motor zero power behavior to BRAKE by default for Autonomous
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders and set motor mode for autonomous
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
     * Implements omniwheel (Mecanum) drive movement using raw power (for TeleOp).
     * @param forward Backward/Forward power (Y axis).
     * @param strafe Left/Right strafing power (X axis).
     * @param rotation Clockwise/Counter-clockwise rotation power.
     */
    public void drive(double forward, double strafe, double rotation) {
        // Ensure that in TeleOp, we use the simple run mode
        if (left_front.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
             setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
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
     * Implements field-relative omniwheel (Mecanum) drive movement using the IMU.
     * @param forward Backward/Forward power (Y axis on the joystick).
     * @param strafe Left/Right strafing power (X axis on the joystick).
     * @param rotation Clockwise/Counter-clockwise rotation power.
     */
    public void driveRelativeField(double forward, double strafe, double rotation) {
        // Add Math.PI (180 degrees) to correct for the Control Hub's new mounting orientation
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI;

        // Rotate the movement vector by the bot's heading
        double robotStrafe = strafe * Math.cos(botHeading) + forward * Math.sin(botHeading);
        double robotForward = -strafe * Math.sin(botHeading) + forward * Math.cos(botHeading);

        // Now, drive the robot using the robot-centric values
        drive(robotForward, robotStrafe, rotation);
    }
    
    /**
     * Drive the robot using encoders for a specified distance and heading.
     * The method calculates target positions and sets the motors to RUN_TO_POSITION.
     * NOTE: This method only sets the target; the OpMode must still wait for the motors to complete.
     * @param forwardInches Distance to move forward/backward (in inches).
     * @param strafeInches Distance to strafe left/right (in inches).
     * @param power The motor power to use (0.0 to 1.0).
     */
    public void driveToPosition(double forwardInches, double strafeInches, double power) {
        int targetLeftFront;
        int targetRightFront;
        int targetLeftBack;
        int targetRightBack;

        // Convert distances to encoder counts
        int forwardCounts = (int) (forwardInches * COUNTS_PER_INCH);
        int strafeCounts = (int) (strafeInches * COUNTS_PER_INCH);

        // Calculate target positions based on current position and desired movement.
        
        // Mecanum kinematics for position:
        // LF: REVERSE motor requires subtraction for FWD motion. FLIPPED TO ADDITION (KINEMATIC FIX).
        targetLeftFront = left_front.getCurrentPosition() + (forwardCounts + strafeCounts); 
        // RF: FORWARD motor requires addition for FWD motion. (CORRECT)
        targetRightFront = right_front.getCurrentPosition() + forwardCounts - strafeCounts; 
        
        // LB: REVERSE motor requires subtraction for FWD motion. FLIPPED TO ADDITION (KINEMATIC FIX).
        targetLeftBack = left_back.getCurrentPosition() + (forwardCounts - strafeCounts); 
        
        // RB: FORWARD motor requires addition for FWD motion. (CORRECT)
        targetRightBack = right_back.getCurrentPosition() + forwardCounts + strafeCounts;

        // Set Target Position
        left_front.setTargetPosition(targetLeftFront);
        right_front.setTargetPosition(targetRightFront);
        left_back.setTargetPosition(targetLeftBack);
        right_back.setTargetPosition(targetRightBack);

        // Switch to RUN_TO_POSITION mode
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        left_front.setPower(Math.abs(power));
        right_front.setPower(Math.abs(power));
        left_back.setPower(Math.abs(power));
        right_back.setPower(Math.abs(power));
    }
    
    /**
     * Checks if all motors have finished their current move to position.
     * @return True if all motors are busy (still moving), False otherwise.
     */
    public boolean isBusy() {
        return left_front.isBusy() || right_front.isBusy() || left_back.isBusy() || right_back.isBusy();
    }

    /**
     * Resets the IMU's yaw angle to zero. This is useful for setting the robot's "forward" direction.
     */
    public void resetIMU() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    public double getHeading(AngleUnit unit) {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(unit);
        }
        return 0;
    }

    /**
     * Get the current run mode of the drive motors (assuming all are the same).
     * @return The DcMotor.RunMode of the left_front motor.
     */
    public DcMotor.RunMode getMode() {
        return left_front.getMode();
    }

    /**
     * Gets the current encoder position of the left_front motor.
     * @return Current encoder position.
     */
    public int getEncoderPosition() {
        return left_front.getCurrentPosition();
    }

    /**
     * Gets the number of inches the robot moves per encoder tick (count).
     * @return Inches per tick.
     */
    public double getInchesPerTick() {
        return 1.0 / COUNTS_PER_INCH;
    }

    /**
     * Stop all motors by setting all drive power to zero.
     */
    public void stop() {
        // Set power to 0.0 before switching to RUN_USING_ENCODER to prevent motor run-on.
        left_front.setPower(0.0);
        right_front.setPower(0.0);
        left_back.setPower(0.0);
        right_back.setPower(0.0);
        
        // Revert to RUN_USING_ENCODER for general utility
        if (left_front.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
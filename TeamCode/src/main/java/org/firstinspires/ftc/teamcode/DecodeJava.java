package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.OmniwheelDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensorDetector;
import org.firstinspires.ftc.teamcode.mechanisms.Separator;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;

/**
 * TeleOp OpMode for driving an omniwheel (Mecanum) robot.
 * This version contains advanced operator controls for sorting and feeding.
 */
@TeleOp(name = "Decode Java Drive", group = "Main")
public class DecodeJava extends LinearOpMode {

    // Instantiate mechanisms
    private OmniwheelDrive driveTrain = new OmniwheelDrive();
    private ColorSensorDetector colorSensor = new ColorSensorDetector();
    private Separator separator = new Separator();
    private Shooter shooter = new Shooter();
    private Intake intake = new Intake();
    private Lift lift = new Lift();

    // Variables for drive mode toggle
    private boolean isFieldCentric = false;
    private boolean aToggleLast = false;

    // Threshold for detecting an artifact
    private static final double ARTIFACT_PRESENCE_DISTANCE_CM = 6.0;

    @Override
    public void runOpMode() {
        // 1. Initialize all mechanisms
        driveTrain.init(hardwareMap);
        colorSensor.init(hardwareMap);
        separator.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Gamepad 2 for Drive, Gamepad 1 for Operator");
        telemetry.update();

        waitForStart();

        driveTrain.resetIMU();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Gamepad 2 controls the robot's movement
            handleDriveControls();
            
            // Gamepad 1 controls all other mechanisms (intake, lift, shooter, etc.)
            handleOperatorControls();

            // Update all telemetry
            telemetry.update();
        }
    }

    /**
     * Handles all robot movement based on gamepad 2 inputs.
     */
    private void handleDriveControls() {
        double forward = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double rotation = gamepad2.right_stick_x;

        boolean aToggle = gamepad2.a;
        if (aToggle && !aToggleLast) {
            isFieldCentric = !isFieldCentric;
        }
        aToggleLast = aToggle;

        if (gamepad2.b) {
            driveTrain.resetIMU();
        }

        if (isFieldCentric) {
            driveTrain.driveRelativeField(forward, strafe, rotation);
        } else {
            driveTrain.drive(forward, strafe, rotation);
        }
        telemetry.addData("Drive Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
    }

    /**
     * Handles all operator controls for mechanisms based on gamepad 1 inputs.
     */
    private void handleOperatorControls() {
        // --- Shooter Control ---
        if (gamepad1.dpad_up) {
            shooter.setPower(0.6);
        } else if (gamepad1.dpad_down) {
            shooter.stop();
        }

        // --- Intake Control ---
        if (gamepad1.b) { // 'B' button runs intake IN
            intake.in(1.0);
        } else if (gamepad1.a) { // 'A' button runs intake OUT
            intake.out(1.0);
        } else {
            intake.stop();
        }

        // --- Lift & Feeder Control ---
        double rightLiftPower = 0;
        double leftLiftPower = 0;

        // The triggers provide a variable-speed manual override to move the lift DOWN.
        // This is useful for clearing jams.
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;

        if (rightTrigger > 0) {
            rightLiftPower = -rightTrigger; // Negative power to move down
        } 
        if (leftTrigger > 0) {
            leftLiftPower = -leftTrigger; // Negative power to move down
        }

        // The bumpers are used to feed artifacts UP into the shooter.
        // This only happens if a trigger is not being pressed.
        if (rightTrigger == 0 && gamepad1.right_bumper && isGreenDetected()) {
            rightLiftPower = 1.0; // Feed green artifact with right lift
            separator.sortRight();
        } else if (leftTrigger == 0 && gamepad1.left_bumper && isPurpleDetected()) {
            leftLiftPower = 1.0; // Feed purple artifact with left lift
            separator.sortLeft();
        } else if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
             // Stop the separator if no feeding action is happening
             separator.stop();
        }
        
        lift.setIndividualPower(rightLiftPower, leftLiftPower);

        // --- Telemetry for Operator ---
        telemetry.addData("Lift Power (L, R)", "%.2f, %.2f", leftLiftPower, rightLiftPower);
        telemetry.addData("Distance (cm)", "%.2f", colorSensor.getDistance(DistanceUnit.CM));
    }

    /**
     * Checks if a green artifact is present and ready to be fed.
     * @return True if a green artifact is detected within range.
     */
    private boolean isGreenDetected() {
        // Check for artifact presence first
        if (colorSensor.getDistance(DistanceUnit.CM) > ARTIFACT_PRESENCE_DISTANCE_CM) {
            return false;
        }
        // Check for green color dominance
        return colorSensor.getGreen() > colorSensor.getBlue();
    }

    /**
     * Checks if a purple (blue) artifact is present and ready to be fed.
     * @return True if a purple artifact is detected within range.
     */
    private boolean isPurpleDetected() {
        // Check for artifact presence first
        if (colorSensor.getDistance(DistanceUnit.CM) > ARTIFACT_PRESENCE_DISTANCE_CM) {
            return false;
        }
        // Check for purple/blue color dominance
        return colorSensor.getBlue() > colorSensor.getGreen();
    }
}
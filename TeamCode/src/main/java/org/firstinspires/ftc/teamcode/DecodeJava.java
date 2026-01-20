package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.OmniwheelDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensorDetector;
import org.firstinspires.ftc.teamcode.mechanisms.Separator;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;

/**
 * TeleOp OpMode for driving an omniwheel (Mecanum) robot.
 * This version includes a revised and more robust auto-intake system.
 */
@TeleOp(name = "Decode Java Drive", group = "Main")
public class DecodeJava extends LinearOpMode {

    // Mechanisms
    private OmniwheelDrive driveTrain = new OmniwheelDrive();
    private ColorSensorDetector colorSensor = new ColorSensorDetector();
    private Separator separator = new Separator();
    private Shooter shooter = new Shooter();
    private Intake intake = new Intake();
    private Lift lift = new Lift();

    // Drive mode state
    private boolean isFieldCentric = false;
    private boolean aButtonGamepad2_last = false; // Fixed: Separate toggle for gamepad 2

    // Auto Intake State Machine
    private enum AutoIntakeState { INACTIVE, STAGING_LEFT, STAGING_RIGHT, STAGING_CENTER, DONE }
    private AutoIntakeState autoIntakeState = AutoIntakeState.INACTIVE;
    private boolean aButtonGamepad1_last = false; // Fixed: Separate toggle for gamepad 1
    private ElapsedTime stateTimer = new ElapsedTime();
    private static final double STAGE_TIMEOUT_S = 3.0;

    private static final double ARTIFACT_PRESENCE_DISTANCE_CM = 6.0;

    @Override
    public void runOpMode() {
        // Initialization
        driveTrain.init(hardwareMap);
        colorSensor.init(hardwareMap);
        separator.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Gamepad 2: Drive | Gamepad 1: Operator");
        telemetry.update();

        waitForStart();
        driveTrain.resetIMU();
        if (isStopRequested()) return;

        // Main Loop
        while (opModeIsActive()) {
            handleDriveControls();

            // Auto-intake mode takes priority over manual operator controls.
            if (autoIntakeState != AutoIntakeState.INACTIVE) {
                runAutoIntake();
            } else {
                handleOperatorControls();
            }
            
            telemetry.addData("Auto Intake State", autoIntakeState.toString());
            telemetry.update();
        }
    }

    private void handleDriveControls() {
        double forward = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double rotation = gamepad2.right_stick_x;

        if (gamepad2.a && !aButtonGamepad2_last) {
            isFieldCentric = !isFieldCentric;
        }
        aButtonGamepad2_last = gamepad2.a;

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

    private void handleOperatorControls() {
        // --- Auto Intake Toggle ---
        if (gamepad1.a && !aButtonGamepad1_last) {
            autoIntakeState = AutoIntakeState.STAGING_LEFT; // Start the sequence
            stateTimer.reset(); // Reset timer for the first stage
        }
        aButtonGamepad1_last = gamepad1.a;
        
        // --- Manual Controls ---
        // Shooter
        if (gamepad1.dpad_up) shooter.setPower(0.65); else if (gamepad1.dpad_down) shooter.stop();

        // Intake
        if (gamepad1.b) intake.in(1.0); else if (gamepad1.x) intake.out(1.0); else intake.stop();

        // Lift & Feeder
        double rightLiftPower = 0;
        double leftLiftPower = 0;

        if (gamepad1.right_trigger > 0) rightLiftPower = -gamepad1.right_trigger;
        if (gamepad1.left_trigger > 0) leftLiftPower = -gamepad1.left_trigger;

        boolean isFeedingRight = gamepad1.right_bumper && gamepad1.right_trigger == 0 && isGreenDetected();
        boolean isFeedingLeft = gamepad1.left_bumper && gamepad1.left_trigger == 0 && isPurpleDetected();

        if (isFeedingRight) {
            rightLiftPower = 1.0; 
            separator.sortRight();
        } else if (isFeedingLeft) {
            leftLiftPower = 1.0;
            separator.sortLeft();
        } else {
            separator.stop();
        }
        
        lift.setIndividualPower(rightLiftPower, leftLiftPower);
    }
    
    private void runAutoIntake() {
        // Manual override to cancel the sequence
        if (gamepad1.y || stateTimer.seconds() > STAGE_TIMEOUT_S) {
            autoIntakeState = AutoIntakeState.INACTIVE;
            intake.stop(); lift.stop(); separator.stop();
            telemetry.addData("Auto-Intake", "CANCELLED");
            return;
        }

        intake.in(1.0); // Keep intake running throughout the sequence

        switch (autoIntakeState) {
            case STAGING_LEFT:
                if (!colorSensor.isStagedLeft()) {
                    lift.setIndividualPower(1.0, 1.0);
                    separator.sortLeft();
                } else {
                    lift.stop(); // Stop lifts before transitioning
                    separator.stop();
                    autoIntakeState = AutoIntakeState.STAGING_RIGHT;
                    stateTimer.reset();
                }
                break;
                
            case STAGING_RIGHT:
                if (!colorSensor.isStagedRight()) {
                    lift.setIndividualPower(1.0, 1.0);
                    separator.sortRight();
                } else {
                    lift.stop();
                    separator.stop();
                    autoIntakeState = AutoIntakeState.STAGING_CENTER;
                    stateTimer.reset();
                }
                break;

            case STAGING_CENTER:
                if (colorSensor.isStagedCenter()) {
                    autoIntakeState = AutoIntakeState.DONE;
                }
                break;
            
            case DONE:
                intake.stop();
                lift.stop();
                separator.stop();
                autoIntakeState = AutoIntakeState.INACTIVE; // Reset for next run
                break;
        }
    }

    private boolean isGreenDetected() {
        if (colorSensor.getDistance(ColorSensorDetector.SensorLocation.CENTER, DistanceUnit.CM) > ARTIFACT_PRESENCE_DISTANCE_CM) return false;
        return colorSensor.getGreen(ColorSensorDetector.SensorLocation.CENTER) > colorSensor.getBlue(ColorSensorDetector.SensorLocation.CENTER);
    }

    private boolean isPurpleDetected() {
        if (colorSensor.getDistance(ColorSensorDetector.SensorLocation.CENTER, DistanceUnit.CM) > ARTIFACT_PRESENCE_DISTANCE_CM) return false;
        return colorSensor.getBlue(ColorSensorDetector.SensorLocation.CENTER) > colorSensor.getGreen(ColorSensorDetector.SensorLocation.CENTER);
    }
}
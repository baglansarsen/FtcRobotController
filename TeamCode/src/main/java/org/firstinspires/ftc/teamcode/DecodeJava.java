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

import static org.firstinspires.ftc.teamcode.mechanisms.Shooter.TARGET_VELOCITY_COUNTS_PER_SEC;

/**
 * TeleOp OpMode for driving an omniwheel (Mecanum) robot.
 * This version includes a sensor-driven auto-shoot system with velocity recovery checks for consistent shots.
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
    private boolean aButtonGamepad2_last = false;

    // Auto Intake State Machine
    private enum AutoIntakeState { INACTIVE, STAGING_LEFT, STAGING_RIGHT, STAGING_CENTER, DONE }
    private AutoIntakeState autoIntakeState = AutoIntakeState.INACTIVE;
    private boolean aButtonGamepad1_last = false;
    private ElapsedTime stateTimer = new ElapsedTime();
    private static final double STAGE_TIMEOUT_S = 3.0;
    
    // Auto Shoot State Machine - Added recovery states
    private enum AutoShootState { INACTIVE, SPIN_UP, FEED_1ST, RECOVER_2ND, FEED_2ND, RECOVER_3RD, FEED_3RD, DONE }
    private AutoShootState autoShootState = AutoShootState.INACTIVE;
    private boolean yButtonGamepad1_last = false;
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double FEED_TIMEOUT_S = 1.5;

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
        telemetry.update();

        waitForStart();
        driveTrain.resetIMU();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (autoShootState != AutoShootState.INACTIVE) {
                runAutoShoot();
            } else {
                handleDriveControls();
                if (autoIntakeState != AutoIntakeState.INACTIVE) {
                    runAutoIntake();
                } else {
                    handleOperatorControls();
                }
            }
            yButtonGamepad1_last = gamepad1.y;
            telemetry.addData("Auto Intake State", autoIntakeState.toString());
            telemetry.addData("Auto Shoot State", autoShootState.toString());
            telemetry.addData("Shooter Ready", shooter.isAtTargetVelocity());
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
        if (gamepad1.y && !yButtonGamepad1_last) {
            autoShootState = AutoShootState.SPIN_UP;
            shootTimer.reset();
        }
        
        if (gamepad1.a && !aButtonGamepad1_last) {
            autoIntakeState = AutoIntakeState.STAGING_LEFT;
            stateTimer.reset();
        }
        aButtonGamepad1_last = gamepad1.a;
        
        if (gamepad1.dpad_up) shooter.setVelocity(0.65 * TARGET_VELOCITY_COUNTS_PER_SEC); else if (gamepad1.dpad_down) shooter.stop();
        if (gamepad1.b) intake.in(1.0); else if (gamepad1.x) intake.out(1.0); else intake.stop();

        double rightLiftPower = 0;
        double leftLiftPower = 0;
        if (gamepad1.right_trigger > 0) rightLiftPower = -gamepad1.right_trigger;
        if (gamepad1.left_trigger > 0) leftLiftPower = -gamepad1.left_trigger;

        boolean isFeedingRight = gamepad1.right_bumper && gamepad1.right_trigger == 0 && isGreenDetected();
        boolean isFeedingLeft = gamepad1.left_bumper && gamepad1.left_trigger == 0 && isPurpleDetected();

        if (isFeedingRight) { rightLiftPower = 1.0; separator.sortRight(); }
        else if (isFeedingLeft) { leftLiftPower = 1.0; separator.sortLeft(); }
        else { separator.stop(); }
        
        lift.setIndividualPower(rightLiftPower, leftLiftPower);
    }
    
    private void runAutoIntake() {
        if (gamepad1.y || stateTimer.seconds() > STAGE_TIMEOUT_S) {
            autoIntakeState = AutoIntakeState.INACTIVE;
            intake.stop(); lift.stop(); separator.stop();
            return;
        }
        intake.in(1.0);
        switch (autoIntakeState) {
            case STAGING_LEFT:
                if (!colorSensor.isStagedLeft()) {
                    lift.setIndividualPower(0.0, 1.0);
                    separator.sortLeft();
                } else {
                    lift.stop(); separator.stop();
                    autoIntakeState = AutoIntakeState.STAGING_RIGHT;
                    stateTimer.reset();
                }
                break;
            case STAGING_RIGHT:
                if (!colorSensor.isStagedRight()) {
                    lift.setIndividualPower(1.0, 0.0);
                    separator.sortRight();
                } else {
                    lift.stop(); separator.stop();
                    autoIntakeState = AutoIntakeState.STAGING_CENTER;
                    stateTimer.reset();
                }
                break;
            case STAGING_CENTER:
                if (colorSensor.isStagedCenter()) autoIntakeState = AutoIntakeState.DONE;
                break;
            case DONE:
                intake.stop(); lift.stop(); separator.stop();
                autoIntakeState = AutoIntakeState.INACTIVE;
                break;
        }
    }
    
    private void runAutoShoot() {
        if (gamepad1.y && !yButtonGamepad1_last) {
            autoShootState = AutoShootState.INACTIVE;
            shooter.stop(); lift.stop(); separator.stop(); intake.stop();
            return;
        }
        driveTrain.drive(0, 0, 0);

        switch (autoShootState) {
            case SPIN_UP:
                shooter.setVelocity(0.65 * TARGET_VELOCITY_COUNTS_PER_SEC);
                if (shooter.isAtTargetVelocity()) {
                    autoShootState = AutoShootState.FEED_1ST;
                    shootTimer.reset();
                }
                break;

            case FEED_1ST:
                lift.setIndividualPower(0.0, 1.0);
                separator.sortLeft();
                intake.in(1.0);
                if ((!colorSensor.isStagedLeft() && shootTimer.seconds() > 0.5) || shootTimer.seconds() > FEED_TIMEOUT_S) {
                    lift.stop();
                    autoShootState = AutoShootState.RECOVER_2ND; // Recover velocity before next shot
                    shootTimer.reset();
                }
                break;

            case RECOVER_2ND:
                lift.stop();
                intake.stop(); // Temporarily stop to reduce load if necessary
                if (shooter.isAtTargetVelocity() || shootTimer.seconds() > 1.0) {
                    autoShootState = AutoShootState.FEED_2ND;
                    shootTimer.reset();
                }
                break;

            case FEED_2ND:
                lift.setIndividualPower(1.0, 0.0);
                separator.sortRight();
                intake.in(1.0);
                if ((!colorSensor.isStagedRight() && shootTimer.seconds() > 0.5) || shootTimer.seconds() > FEED_TIMEOUT_S) {
                    lift.stop();
                    autoShootState = AutoShootState.RECOVER_3RD; // Recover velocity before next shot
                    shootTimer.reset();
                }
                break;

            case RECOVER_3RD:
                lift.stop();
                intake.stop();
                if (shooter.isAtTargetVelocity() || shootTimer.seconds() > 1.0) {
                    autoShootState = AutoShootState.FEED_3RD;
                    shootTimer.reset();
                }
                break;
                
            case FEED_3RD:
                lift.up();
                separator.sortLeft();
                intake.in(1.0);
                if ((!colorSensor.isStagedCenter() && shootTimer.seconds() > 0.7) || shootTimer.seconds() > FEED_TIMEOUT_S + 0.5) {
                     autoShootState = AutoShootState.DONE;
                     shootTimer.reset();
                }
                break;

            case DONE:
                if (shootTimer.seconds() > 0.6) {
                    shooter.stop(); lift.stop(); separator.stop(); intake.stop();
                    autoShootState = AutoShootState.INACTIVE;
                }
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
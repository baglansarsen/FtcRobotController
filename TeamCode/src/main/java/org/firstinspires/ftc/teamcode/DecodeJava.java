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
 * Final version with simplified, reliable auto-shoot logic now that wiring is fixed.
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

    // Auto Intake State machine and control
    private enum AutoIntakeState { INACTIVE, RUNNING }
    private AutoIntakeState autoIntakeState = AutoIntakeState.INACTIVE;
    private boolean aButtonGamepad1_last = false;
    private boolean isLiftingForCenter = false;
    private ElapsedTime centerLiftTimer = new ElapsedTime();
    private static final double CENTER_LIFT_TIMEOUT_S = 2.0;

    // Auto Shoot State Machine - Simplified with velocity recovery
    private enum AutoShootState { INACTIVE, SPIN_UP, FEED_1ST, RECOVER_2ND, FEED_2ND, RECOVER_3RD, FEED_3RD, DONE }
    private AutoShootState autoShootState = AutoShootState.INACTIVE;
    private boolean yButtonGamepad1_last = false;
    private ElapsedTime shootTimer = new ElapsedTime();

    private static final double SHOOT_VELOCITY = 1170; // 0.65 * 1800
    private static final double ARTIFACT_PRESENCE_DISTANCE_CM = 6.0;


    @Override
    public void runOpMode() {
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
            aButtonGamepad1_last = gamepad1.a;

            // Calculate current ball count for telemetry
            int ballCount = 0;
            if (colorSensor.isStagedLeft()) ballCount++;
            if (colorSensor.isStagedRight()) ballCount++;
            if (colorSensor.isStagedCenter()) ballCount++;

            telemetry.addData("Auto State", autoShootState != AutoShootState.INACTIVE ? autoShootState.toString() : (autoIntakeState != AutoIntakeState.INACTIVE ? "AUTO INTAKE" : "MANUAL"));
            telemetry.addData("Balls Staged", ballCount);
            telemetry.addData("Staging Status", "L:%b R:%b C:%b", 
                    colorSensor.isStagedLeft(), colorSensor.isStagedRight(), colorSensor.isStagedCenter());
            telemetry.addData("Sensor CM", "L:%.1f R:%.1f C:%.1f",
                    colorSensor.getDistance(ColorSensorDetector.SensorLocation.LEFT, DistanceUnit.CM),
                    colorSensor.getDistance(ColorSensorDetector.SensorLocation.RIGHT, DistanceUnit.CM),
                    colorSensor.getDistance(ColorSensorDetector.SensorLocation.CENTER, DistanceUnit.CM));
            telemetry.addData("Shooter Ready", shooter.isAtTargetVelocity());
            telemetry.update();
        }
    }

    private void handleDriveControls() {
        double forward = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double rotation = gamepad2.right_stick_x;
        if (gamepad2.a && !aButtonGamepad2_last) isFieldCentric = !isFieldCentric;
        aButtonGamepad2_last = gamepad2.a;
        if (gamepad2.b) driveTrain.resetIMU();
        if (isFieldCentric) driveTrain.driveRelativeField(forward, strafe, rotation); else driveTrain.drive(forward, strafe, rotation);
    }

    private void handleOperatorControls() {
        if (gamepad1.y && !yButtonGamepad1_last) {
            autoShootState = AutoShootState.SPIN_UP;
            shootTimer.reset(); 
        }
        
        // Toggle Auto Intake State
        if (gamepad1.a && !aButtonGamepad1_last) {
            if (autoIntakeState == AutoIntakeState.RUNNING) {
                // Stop and reset state
                autoIntakeState = AutoIntakeState.INACTIVE;
                isLiftingForCenter = false;
                intake.stop();
                lift.stop();
                separator.stop();
            } else {
                // Start and reset state
                autoIntakeState = AutoIntakeState.RUNNING;
                isLiftingForCenter = false;
            }
        }
        
        // Manual Shooter Controls
        if (gamepad1.dpad_up) shooter.setVelocity(SHOOT_VELOCITY); else if (gamepad1.dpad_down) shooter.stop();
        
        // Manual Intake and Sorting Controls
        if (gamepad1.b) {
            intake.in(1.0);
            if (colorSensor.isStagedLeft()) {
                separator.sortRight();
            } else {
                separator.sortLeft();
            }
        } else if (gamepad1.x) {
            intake.out(1.0);
            separator.stop();
        } else {
            intake.stop();
            separator.stop();
        }
        
        // Manual Lift Controls
        double rPower = gamepad1.left_bumper ? -1.0 : gamepad1.left_trigger;
        double lPower = gamepad1.right_bumper ? -1.0 : gamepad1.right_trigger;
        lift.setIndividualPower(rPower, lPower);
    }
    
    private void runAutoIntake() {
        // Interrupt logic: gamepad1.x (outtake) or gamepad1.y (shoot sequence)
        if (gamepad1.x || gamepad1.y) {
            autoIntakeState = AutoIntakeState.INACTIVE;
            isLiftingForCenter = false; // Reset on interrupt
            intake.stop(); lift.stop(); separator.stop();
            return;
        }

        intake.in(1.0); 

        // Logic for lifting the 3rd ball with a timeout
        if (isLiftingForCenter) {
            if (colorSensor.isStagedCenter() || centerLiftTimer.seconds() > CENTER_LIFT_TIMEOUT_S) {
                // Ball is staged or we timed out, stop everything.
                intake.stop();
                lift.stop();
                separator.stop();
                autoIntakeState = AutoIntakeState.INACTIVE;
                isLiftingForCenter = false;
            }
            // Otherwise, do nothing and let the lift continue running.
            return; 
        }

        // Standard staging logic for 1st and 2nd balls
        if (!colorSensor.isStagedLeft()) {
            lift.setIndividualPower(0.0, 1.0);
            separator.sortLeft();
        } else if (!colorSensor.isStagedRight()) {
            lift.setIndividualPower(1.0, 0.0);
            separator.sortRight();
        } else if (!colorSensor.isStagedCenter()) {
            // L and R are full, C is not. Start lifting for the center ball.
            isLiftingForCenter = true;
            centerLiftTimer.reset();
            lift.setIndividualPower(-0.2, -0.2); // Run both lifts up
            separator.stop();
        } else {
            // All 3 are somehow already staged when starting. Stop immediately.
            autoIntakeState = AutoIntakeState.INACTIVE;
            intake.stop();
            lift.stop();
            separator.stop();
        }
    }
    
    private void runAutoShoot() {
        // ... (runAutoShoot logic remains unchanged)
        if (gamepad1.y && !yButtonGamepad1_last) {
            autoShootState = AutoShootState.INACTIVE;
            shooter.stop(); lift.stop(); separator.stop(); intake.stop();
            return;
        }
        driveTrain.drive(0, 0, 0);

        switch (autoShootState) {
            case SPIN_UP:
                shooter.setVelocity(SHOOT_VELOCITY);
                if (shooter.isAtTargetVelocity()) {
                    autoShootState = AutoShootState.FEED_1ST;
                    shootTimer.reset(); 
                }
                break;

            case FEED_1ST:
                lift.setIndividualPower(0, 1.0);
                separator.sortLeft();
                intake.in(1.0);
                if (shootTimer.seconds() > 0.5) {
                    lift.stop(); intake.stop();
                    autoShootState = AutoShootState.RECOVER_2ND;
                }
                break;

            case RECOVER_2ND:
                lift.stop(); separator.stop(); intake.stop();
                if (shooter.isAtTargetVelocity()) {
                    autoShootState = AutoShootState.FEED_2ND;
                    shootTimer.reset(); 
                }
                break;

            case FEED_2ND:
                lift.setIndividualPower(1.0, 0);
                separator.sortLeft(); 
                intake.in(1.0);
                if (shootTimer.seconds() > 0.5) {
                    lift.stop(); intake.stop();
                    autoShootState = AutoShootState.RECOVER_3RD;
                }
                break;

            case RECOVER_3RD:
                lift.stop(); separator.stop(); intake.stop();
                if (shooter.isAtTargetVelocity()) {
                    autoShootState = AutoShootState.FEED_3RD;
                    shootTimer.reset(); 
                }
                break;
                
            case FEED_3RD:
                lift.up();
                separator.sortLeft();
                intake.in(1.0);
                if (shootTimer.seconds() > 1.0) {
                    lift.stop(); separator.stop(); intake.stop();
                    autoShootState = AutoShootState.DONE;
                }
                break;

            case DONE:
                shooter.stop(); lift.stop(); separator.stop(); intake.stop();
                autoShootState = AutoShootState.INACTIVE;
                break;
        }
    }
}

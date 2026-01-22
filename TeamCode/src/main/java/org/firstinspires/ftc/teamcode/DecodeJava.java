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
 * Final version with simplified, reliable auto-shoot logic now that wiring is fixed and auto-intake is updated for 3 balls.
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
    // Updated states to manage 3 balls sequentially: Stage Left, Stage Right, Stage Center, then wait to shoot.
    private enum AutoIntakeState { INACTIVE, STAGE_ONE, STAGE_TWO, STAGE_THREE, WAITING_TO_SHOOT }
    private AutoIntakeState autoIntakeState = AutoIntakeState.INACTIVE;
    private boolean aButtonGamepad1_last = false;
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
            } else if (autoIntakeState == AutoIntakeState.WAITING_TO_SHOOT) {
                // If auto intake finished staging all 3 balls, transition to auto shoot
                autoShootState = AutoShootState.SPIN_UP;
                shootTimer.reset();
                runAutoShoot(); // Immediately run auto shoot state to start spin-up
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

            String autoModeDisplay = "MANUAL";
            if (autoShootState != AutoShootState.INACTIVE) {
                autoModeDisplay = "AUTO SHOOT: " + autoShootState.toString();
            } else if (autoIntakeState == AutoIntakeState.WAITING_TO_SHOOT) {
                autoModeDisplay = "AUTO INTAKE: READY TO SHOOT";
            } else if (autoIntakeState != AutoIntakeState.INACTIVE) {
                autoModeDisplay = "AUTO INTAKE: " + autoIntakeState.toString();
            }
            
            telemetry.addData("Auto State", autoModeDisplay);
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
        // Start Auto Shoot sequence manually (interrupts intake)
        if (gamepad1.y && !yButtonGamepad1_last) {
            autoShootState = AutoShootState.SPIN_UP;
            shootTimer.reset(); 
            autoIntakeState = AutoIntakeState.INACTIVE; // Stop intake if running
        }
        
        // Toggle Auto Intake State (Only if not in auto shoot)
        if (gamepad1.a && !aButtonGamepad1_last) {
            if (autoIntakeState == AutoIntakeState.INACTIVE) {
                // Start Auto Intake sequence
                autoIntakeState = AutoIntakeState.STAGE_ONE;
                intake.in(1.0); // Start intake immediately
            } else if (autoIntakeState == AutoIntakeState.WAITING_TO_SHOOT) {
                // Stop and reset state if ready to shoot but shoot hasn't started
                autoIntakeState = AutoIntakeState.INACTIVE;
                intake.stop();
                lift.stop();
                separator.stop();
            } else {
                // Stop mid-sequence (STAGE_ONE/TWO/THREE)
                autoIntakeState = AutoIntakeState.INACTIVE;
                intake.stop();
                lift.stop();
                separator.stop();
            }
        }
        
        // Manual Shooter Controls
        if (gamepad1.dpad_up) shooter.setVelocity(SHOOT_VELOCITY); else if (gamepad1.dpad_down) shooter.stop();
        
        // Manual Intake and Sorting Controls (Only if not in auto intake/shoot)
        if (autoIntakeState == AutoIntakeState.INACTIVE && autoShootState == AutoShootState.INACTIVE) {
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
        }
        
        // Manual Lift Controls (Only if not in auto intake/shoot)
        if (autoIntakeState == AutoIntakeState.INACTIVE && autoShootState == AutoShootState.INACTIVE) {
            double rPower = gamepad1.left_bumper ? -1.0 : gamepad1.left_trigger;
            double lPower = gamepad1.right_bumper ? -1.0 : gamepad1.right_trigger;
            lift.setIndividualPower(rPower, lPower);
        }
    }
    
    private void runAutoIntake() {
        // Interrupt logic: gamepad1.x (outtake) or gamepad1.y (shoot sequence) will stop intake via handleOperatorControls/runOpMode
        if (gamepad1.x || gamepad1.y) {
            autoIntakeState = AutoIntakeState.INACTIVE;
            intake.stop(); lift.stop(); separator.stop();
            return;
        }

        intake.in(1.0); // Keep intake running while trying to stage

        switch (autoIntakeState) {
            case STAGE_ONE:
                // Stage 1: Ball 1 goes to Left sensor. If staged, move to STAGE_TWO.
                // Logic based on previous code: lift.setIndividualPower(RightMotorPower, LeftMotorPower)
                lift.setIndividualPower(0.0, 1.0); // Based on old code for staging left
                separator.sortLeft();

                if (colorSensor.isStagedLeft()) {
                    // Ball 1 staged. Stop moving lift for this ball, prepare for next.
                    lift.stop();
                    autoIntakeState = AutoIntakeState.STAGE_TWO;
                    separator.stop(); // Stop sorting while transition to next stage
                }
                break;
                
            case STAGE_TWO:
                // Stage 2: Ball 2 goes to Right sensor. If staged, move to STAGE_THREE.
                lift.setIndividualPower(1.0, 0.0); // Based on old code for staging right
                separator.sortRight();

                if (colorSensor.isStagedRight()) {
                    // Ball 2 staged. Stop moving lift for this ball, prepare for next.
                    lift.stop();
                    autoIntakeState = AutoIntakeState.STAGE_THREE;
                    separator.stop(); // Stop sorting while transition to next stage
                }
                break;
                
            case STAGE_THREE:
                // Stage 3: Ball 3 goes to Center sensor. If staged, transition to WAITING_TO_SHOOT.
                // Use lift.up() as it was used in previous logic for this scenario.
                lift.up(); 
                separator.stop(); // Center ball staging should stop sorting mechanism

                if (colorSensor.isStagedCenter()) {
                    // All 3 balls staged successfully. Stop motors and wait for shoot command (or auto-trigger).
                    intake.stop();
                    lift.stop();
                    separator.stop();
                    autoIntakeState = AutoIntakeState.WAITING_TO_SHOOT;
                } else if (centerLiftTimer.seconds() > CENTER_LIFT_TIMEOUT_S) {
                     // Timeout safety break
                    intake.stop();
                    lift.stop();
                    separator.stop();
                    autoIntakeState = AutoIntakeState.INACTIVE;
                    telemetry.addData("Auto Intake", "Timed out staging 3rd ball.");
                }
                break;
                
            case INACTIVE:
            case WAITING_TO_SHOOT:
                // Should be handled in runOpMode loop, but stop motors if state persists here somehow
                intake.stop(); lift.stop(); separator.stop();
                break;
        }
    }
    
    private void runAutoShoot() {
        // Manual interrupt to stop shooting sequence
        if (gamepad1.y && !yButtonGamepad1_last) {
            autoShootState = AutoShootState.INACTIVE;
            shooter.stop(); lift.stop(); separator.stop(); intake.stop();
            autoIntakeState = AutoIntakeState.INACTIVE; // Reset intake state as well
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
                lift.setIndividualPower(0, 1.0); // Based on old code for 1st ball feed
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
                lift.setIndividualPower(1.0, 0); // Based on old code for 2nd ball feed
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
                autoIntakeState = AutoIntakeState.INACTIVE; // Reset intake state after shooting sequence is complete
                break;
        }
    }
}
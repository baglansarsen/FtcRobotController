package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.OmniwheelDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Separator;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensorDetector;

@Autonomous(name = "Basic Shoot & Strafe", group = "Main")
public class AutonomousMode extends LinearOpMode {

    // Mechanisms
    private OmniwheelDrive driveTrain = new OmniwheelDrive();
    private Shooter shooter = new Shooter();
    private Lift lift = new Lift();
    private Separator separator = new Separator();
    private Intake intake = new Intake();
    private ColorSensorDetector colorSensor = new ColorSensorDetector(); // Even if not used now, it's good practice

    // Constants
    private static final double DRIVE_SPEED = 0.7;
    private static final double FORWARD_DURATION_S = 1; // Estimate for 1.5m @ 50% power
    private static final double STRAFE_DURATION_S = 0.5;  // Estimate for 0.2m @ 50% power
    private static final double SHOOT_VELOCITY = 1170;   // Consistent velocity
    private static final double FEED_TIME_S = 0.3; // Duration for feed mechanism to push one artifact
    private static final double RECOVER_TIMEOUT_S = 1.0; // Max time to wait for velocity recovery

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialization
        driveTrain.init(hardwareMap);
        shooter.init(hardwareMap);
        lift.init(hardwareMap);
        separator.init(hardwareMap);
        intake.init(hardwareMap);
        colorSensor.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        driveTrain.resetIMU();
        runtime.reset();

        // 1. Move Forward 1.5m
        driveTrain.drive(DRIVE_SPEED, 0, 0); // Forward movement
        while (opModeIsActive() && runtime.seconds() < FORWARD_DURATION_S) {
            telemetry.addData("Phase", "Moving Forward");
            telemetry.update();
        }
        driveTrain.stop();

        // 2. Full Shooting Sequence
        runShootingSequence();

        // 3. Strafe Left 0.2m
        runtime.reset();
        driveTrain.drive(0, DRIVE_SPEED, 0); // Negative strafe for left
        while (opModeIsActive() && runtime.seconds() < STRAFE_DURATION_S) {
            telemetry.addData("Phase", "Strafing Left");
            telemetry.update();
        }
        driveTrain.stop();

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
    
    /**
     * Executes the three-artifact shooting sequence.
     */
    private void runShootingSequence() {
        if (isStopRequested()) return;
        telemetry.addData("Phase", "Shooter: Spinning Up");
        telemetry.update();

        // Spin up shooter
        shooter.setVelocity(SHOOT_VELOCITY);
        runtime.reset();
        while (opModeIsActive() && !shooter.isAtTargetVelocity() && runtime.seconds() < 2.5) {
            // Wait for shooter to reach target velocity
            telemetry.addData("Status", "Spinning up...");
            telemetry.update();
        }

        // Check if we reached speed or timed out
        if (!opModeIsActive() || !shooter.isAtTargetVelocity()) {
            shooter.stop();
            return;
        }

        // Sequence: Feed 1 -> Recover -> Feed 2 -> Recover -> Feed 3
        feedAndRecover(0, 1.0, Separator.Direction.LEFT);
        feedAndRecover(1.0, 0, Separator.Direction.RIGHT);
        
        // Final shot has no recovery state
        feedFinalShot();

        shooter.stop();
        lift.stop();
        separator.stop();
        intake.stop();
    }
    
    /**
     * Helper for feeding one artifact and waiting for velocity recovery.
     */
    private void feedAndRecover(double rightPower, double leftPower, Separator.Direction separatorDir) {
        if (isStopRequested()) return;
        
        // --- 1. Feed Artifact ---
        lift.setIndividualPower(rightPower, leftPower);
        if (separatorDir == Separator.Direction.LEFT) separator.sortLeft(); else separator.sortRight();
        intake.in(1.0); // Use intake to assist feed
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < FEED_TIME_S) {
            telemetry.addData("Phase", "Shooter: Feeding");
            telemetry.update();
        }
        lift.stop();

        // --- 2. Recover Velocity ---
        lift.stop();
        separator.stop();
        intake.stop();
        runtime.reset();
        while (opModeIsActive() && !shooter.isAtTargetVelocity() && runtime.seconds() < RECOVER_TIMEOUT_S) {
            telemetry.addData("Phase", "Shooter: Recovering Velocity");
            telemetry.addData("Actual Vel", "%.2f", shooter.getVelocity());
            telemetry.update();
        }
    }
    
    /**
     * Helper for the final, central artifact feed.
     */
    private void feedFinalShot() {
        if (isStopRequested()) return;
        
        // Final Shot: Both Lifts + Neutral Separator for center shot
        lift.up();
        separator.stop(); // Use separator.stop() for a straight shot
        intake.in(1.0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < FEED_TIME_S * 2) { // Use double the feed time
            telemetry.addData("Phase", "Shooter: Feeding Final");
            telemetry.update();
        }
        lift.stop();
        separator.stop(); // Ensure all mechanisms stop after feed time
        intake.stop();
    }
}
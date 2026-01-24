package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.OmniwheelDrive;

@Autonomous(name = "Drive Troubleshooter", group = "Tests")
public class DriveTroubleshooter extends LinearOpMode {

    private OmniwheelDrive driveTrain = new OmniwheelDrive();
    private ElapsedTime runtime = new ElapsedTime();

    // --- FSM States for testing ---
    private enum TestState {
        INIT,
        DRIVE_FORWARD_START,
        DRIVE_FORWARD_WAIT,
        TURN_TEST_START,
        TURN_TEST_WAIT,
        STRAFE_TEST_START,
        STRAFE_TEST_WAIT,
        DONE
    }
    private TestState currentState = TestState.INIT;

    // --- Constants ---
    private static final double FORWARD_DISTANCE_IN = 24;
    private static final double STRAFE_DISTANCE_IN = 18;
    private static final double DRIVE_POWER = 0.3;
    private static final double TURN_SPEED = 0.15; // REDUCED for stability
    private static final double TARGET_TURN_DEGREES = 0; 
    private static final double TURN_TOLERANCE_DEGREES = 5.0; // INCREASED tolerance

    @Override
    public void runOpMode() {
        driveTrain.init(hardwareMap);
        telemetry.addData("Status", "Initialized. Press START to begin tests.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        driveTrain.resetIMU();
        runtime.reset();
        currentState = TestState.DRIVE_FORWARD_START;

        while (opModeIsActive() && currentState != TestState.DONE) {
            switch (currentState) {
                case DRIVE_FORWARD_START:
                    // Test 1: Forward movement
                    telemetry.log().add("TEST 1: Starting to drive FORWARD %.1f inches...", FORWARD_DISTANCE_IN);
                    driveTrain.driveToPosition(FORWARD_DISTANCE_IN, 0, DRIVE_POWER);
                    currentState = TestState.DRIVE_FORWARD_WAIT;
                    break;

                case DRIVE_FORWARD_WAIT:
                    if (!driveTrain.isBusy() || isStopRequested()) {
                        driveTrain.stop(); // Stop handles resetting the run mode
                        telemetry.log().add("TEST 1: Drive FORWARD complete.");
                        currentState = TestState.TURN_TEST_START;
                    }
                    break;
                
                case TURN_TEST_START:
                    // Test 2: Turning (using power/IMU)
                    telemetry.log().add("TEST 2: Starting to turn to %.1f degrees (Tol: %.1f)...", TARGET_TURN_DEGREES, TURN_TOLERANCE_DEGREES);
                    // Start turn by setting the correct motor mode
                    driveTrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currentState = TestState.TURN_TEST_WAIT;
                    break;

                case TURN_TEST_WAIT:
                    double currentHeading = driveTrain.getHeading(AngleUnit.DEGREES);
                    double error = TARGET_TURN_DEGREES - currentHeading;
                    while (error > 180)  error -= 360;
                    while (error <= -180) error += 360;

                    if (Math.abs(error) > TURN_TOLERANCE_DEGREES) { // Use new tolerance
                        // Continue turning
                        double rotationPower = Math.signum(error) * TURN_SPEED;
                        driveTrain.drive(0, 0, rotationPower); // drive uses RUN_WITHOUT_ENCODER
                    } else {
                        // Turn complete
                        driveTrain.stop();
                        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset to encoder mode
                        telemetry.log().add("TEST 2: Turn complete. Final Heading: %.2f", currentHeading);
                        currentState = TestState.STRAFE_TEST_START;
                    }
                    break;
                    
                case STRAFE_TEST_START:
                    // Test 3: Strafing movement
                    telemetry.log().add("TEST 3: Starting to STRAFE RIGHT %.1f inches...", STRAFE_DISTANCE_IN);
                    driveTrain.driveToPosition(0, STRAFE_DISTANCE_IN, DRIVE_POWER);
                    currentState = TestState.STRAFE_TEST_WAIT;
                    break;

                case STRAFE_TEST_WAIT:
                    if (!driveTrain.isBusy() || isStopRequested()) {
                        driveTrain.stop();
                        telemetry.log().add("TEST 3: Strafe RIGHT complete.");
                        currentState = TestState.DONE;
                    }
                    break;

                case INIT:
                case DONE:
                    break;
            }

            telemetry.addData("Current State", currentState.toString());
            telemetry.addData("Heading", "%.2f", driveTrain.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        
        // Final shutdown
        driveTrain.stop();
        telemetry.log().add("Drive Troubleshooter finished. Stop OpMode.");
        telemetry.update();
        sleep(2000);
    }
}
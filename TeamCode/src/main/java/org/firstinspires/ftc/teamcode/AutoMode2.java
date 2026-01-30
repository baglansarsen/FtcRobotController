package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensorDetector;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.OmniwheelDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Separator;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name = "near red", group = "Main")
public class AutoMode2 extends LinearOpMode {

    // --- FSM States ---
    private enum State {
        INIT,
        DRIVE_FORWARD_1,
        SHOOT_SEQUENCE,
        STOP_SHOOTER,
        DRIVE_STRAFE,
        DONE
    }
    private State currentState = State.INIT;

    // --- Mechanisms ---
    private OmniwheelDrive driveTrain = new OmniwheelDrive();
    private Shooter shooter = new Shooter();
    private Lift lift = new Lift();
    private Separator separator = new Separator();
    private Intake intake = new Intake();
    private ColorSensorDetector colorSensor = new ColorSensorDetector();

    // --- Constants ---
    private static final double DRIVE_POWER = OmniwheelDrive.DRIVE_SPEED;
    private static final double FORWARD_DISTANCE_IN = 60;
    private static final double STRAFE_DISTANCE_IN = -30;

    private static final double SHOOT_VELOCITY = 1150;
    private static final double FEED_TIME_S = 0.3;
    private static final double FEED_TIME_3RD_S = 0.8;
    private static final double SPINUP_TIMEOUT_S = 2.5;

    private ElapsedTime runtime = new ElapsedTime();

    // --- Reusable Sequence States ---
    private enum ShootState { SPIN_UP, FEED_1ST, RECOVER_2ND, FEED_2ND, RECOVER_3RD, FEED_3RD, COMPLETE }
    private ShootState currentShootState = ShootState.SPIN_UP;

    // Flag for clean FSM state transition: ensures a command (drive, turn, feed, etc.) only starts once.
    private boolean isCommandStarted = false;

    @Override
    public void runOpMode() {
        // Initialization
        driveTrain.init(hardwareMap);
        shooter.init(hardwareMap);
        lift.init(hardwareMap);
        separator.init(hardwareMap);
        intake.init(hardwareMap);
        colorSensor.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Current State: " + currentState.toString());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        driveTrain.resetIMU();
        runtime.reset();
        currentState = State.DRIVE_FORWARD_1;
        currentShootState = ShootState.SPIN_UP; // Initialize shoot sequence state

        while (opModeIsActive() && currentState != State.DONE) {

            switch (currentState) {
                case DRIVE_FORWARD_1:
                    handleDrive(FORWARD_DISTANCE_IN, 0, DRIVE_POWER, State.SHOOT_SEQUENCE);
                    break;

                case SHOOT_SEQUENCE:
                    if (shootSequence() == ShootState.COMPLETE) {
                        currentState = State.STOP_SHOOTER;
                    }
                    break;

                case STOP_SHOOTER:
                    shooter.stop();
                    currentState = State.DRIVE_STRAFE;
                    break;

                case DRIVE_STRAFE:
                    // Strafe right (positive strafeInches)
                    handleDrive(0, STRAFE_DISTANCE_IN, DRIVE_POWER, State.DONE);
                    break;

                case INIT:
                case DONE:
                    // Stop motion on exit
                    break;
            }

            telemetry.addData("Current State", currentState.toString());
            if (currentState == State.SHOOT_SEQUENCE) {
                telemetry.addData("Shoot Sub-State", currentShootState.toString());
            }
            telemetry.addData("Heading (IMU)", "%.2f", driveTrain.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Shooter Velocity", "%.2f", shooter.getVelocity());
            telemetry.addData("Command Started", isCommandStarted); // For debugging
            telemetry.update();
        }

        // Clean shutdown
        driveTrain.stop();
        shooter.stop();
        lift.stop();
        separator.stop();
        intake.stop();
    }

    /**
     * Executes the entire 3-ball shoot sequence.
     * @return The final state of the sequence (COMPLETE when done).
     */
    private ShootState shootSequence() {
        switch (currentShootState) {
            case SPIN_UP:
                handleSpinUp(ShootState.FEED_1ST, SPINUP_TIMEOUT_S);
                break;
            case FEED_1ST:
                handleFeed(0, 1.0, Separator.Direction.LEFT, FEED_TIME_S, ShootState.RECOVER_2ND);
                break;
            case RECOVER_2ND:
                handleRecover(ShootState.FEED_2ND);
                break;
            case FEED_2ND:
                handleFeed(1.0, 0, Separator.Direction.RIGHT, FEED_TIME_S, ShootState.RECOVER_3RD);
                break;
            case RECOVER_3RD:
                handleRecover(ShootState.FEED_3RD);
                break;
            case FEED_3RD:
                // Note: The third feed is longer to ensure it clears the lift mechanism
                handleFeed(0, 1.0, Separator.Direction.LEFT, FEED_TIME_3RD_S, ShootState.COMPLETE);
                break;
            case COMPLETE:
                // Clean up is handled inside handleFeed, just return the state
                break;
        }
        return currentShootState;
    }

    // --- Movement/Mechanism Handlers (Used by both AutonomousMode and reusable sequences) ---

    // Updated handleDrive to take power as an argument
    private void handleDrive(double forwardInches, double strafeInches, double power, State nextState) {
        if (!isCommandStarted) {
            if (driveTrain.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            driveTrain.driveToPosition(forwardInches, strafeInches, power);
            isCommandStarted = true;
        }

        if (!driveTrain.isBusy() || isStopRequested()) {
            driveTrain.stop();
            isCommandStarted = false;
            currentState = nextState;
        }
    }

    // --- Shoot Sequence Handlers ---

    private void handleSpinUp(ShootState nextState, double timeout) {
        if (!isCommandStarted) {
            shooter.setVelocity(SHOOT_VELOCITY);
            runtime.reset();
            isCommandStarted = true;
        }

        if (shooter.isAtTargetVelocity() || runtime.seconds() > timeout || isStopRequested()) {
            isCommandStarted = false;
            currentShootState = nextState;
            runtime.reset();
        }
    }

    private void handleFeed(double rightLiftPower, double leftLiftPower, Separator.Direction separatorDir, double duration, ShootState nextState) {
        if (!isCommandStarted) {
            runtime.reset();
            isCommandStarted = true;
        }

        lift.setIndividualPower(rightLiftPower, leftLiftPower);
        if (separatorDir == Separator.Direction.LEFT) {
            separator.sortLeft();
        } else if (separatorDir == Separator.Direction.RIGHT) {
            separator.sortRight();
        } else {
            separator.stop();
        }
        intake.in(1.0);

        if (runtime.seconds() > duration || isStopRequested()) {
            lift.stop();
            separator.stop();
            intake.stop();
            isCommandStarted = false;
            currentShootState = nextState;
            runtime.reset();
        }
    }

    private void handleRecover(ShootState nextState) {
        // Recovery is just waiting for shooter velocity to be stable again
        if (shooter.isAtTargetVelocity() || isStopRequested()) {
            currentShootState = nextState;
            runtime.reset();
        }
    }
}

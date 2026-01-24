package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.OmniwheelDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Separator;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensorDetector;

@Autonomous(name = "Basic Shoot & Strafe (FSM & Enc)", group = "Main")
public class AutonomousMode extends LinearOpMode {

    // --- FSM States ---
    private enum State {
        INIT,
        DRIVE_FORWARD_1,
        SHOOT_SEQUENCE, 
        STOP_SHOOTER,       // New state to stop the shooter
        DRIVE_STRAFE,       // New state to strafe right
        TURN_LEFT,          // New state to turn
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
    private static final double STRAFE_DISTANCE_IN = 20;
    private static final double TURN_DEGREES = -145; // right turn
    private static final double SHOOT_VELOCITY = 1170;
    private static final double FEED_TIME_S = 0.3;
    private static final double FEED_TIME_3RD_S = 0.9;
    private static final double SPINUP_TIMEOUT_S = 2.5;
    private static final double TURN_SPEED = 1;
    private static final double TURN_TOLERANCE_DEGREES = 5.0; 

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
            // Note: isCommandStarted is reset inside the handler when a command completes.
            switch (currentState) {
                case DRIVE_FORWARD_1:
                    handleDrive(FORWARD_DISTANCE_IN, 0, State.SHOOT_SEQUENCE);
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
                    handleDrive(0, STRAFE_DISTANCE_IN, State.TURN_LEFT);
                    break;

                case TURN_LEFT:
                    handleTurn(TURN_DEGREES, State.DONE);
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
            telemetry.addData("Heading", "%.2f", driveTrain.getHeading(AngleUnit.DEGREES));
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

    private void handleDrive(double forwardInches, double strafeInches, State nextState) {
        if (!isCommandStarted) {
            if (driveTrain.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            driveTrain.driveToPosition(forwardInches, strafeInches, DRIVE_POWER);
            isCommandStarted = true;
        }

        if (!driveTrain.isBusy() || isStopRequested()) {
            driveTrain.stop(); 
            isCommandStarted = false; 
            currentState = nextState;
        }
    }

    private void handleTurn(double degrees, State nextState) {
        if (!isCommandStarted) {
            driveTrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            isCommandStarted = true;
        }

        // Calculate error and normalize it to [-180, 180] for shortest turn path
        double error = degrees - driveTrain.getHeading(AngleUnit.DEGREES);
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;
        
        telemetry.addData("degree", degrees); // For debugging
        telemetry.addData("Error", error); // For debugging
        telemetry.addData("headinggg", driveTrain.getHeading(AngleUnit.DEGREES)); // For debugging

        if (Math.abs(error) > TURN_TOLERANCE_DEGREES) {
            // Invert the sign of the applied power because the motor/IMU setup seems to have an inverted direction.
            driveTrain.drive(0, 0, -Math.signum(error) * TURN_SPEED);
        } else {
            driveTrain.stop();
            driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
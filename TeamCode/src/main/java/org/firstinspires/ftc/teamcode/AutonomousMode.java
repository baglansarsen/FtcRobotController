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
        SPIN_UP_SHOOTER,
        FEED_1ST,
        RECOVER_2ND,
        FEED_2ND,
        RECOVER_3RD,
        FEED_3RD,
        TURN_TO_INTAKE,
        INTAKE_BALLS,
        DRIVE_BACK_TO_SHOOT,
        TURN_TO_SHOOT,
        SHOOT_COLLECTED_BALLS,
        RECOVER_COLLECTED_2,
        SHOOT_COLLECTED_2,
        RECOVER_COLLECTED_3,
        SHOOT_COLLECTED_3,
        DRIVE_STRAFE_2,
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
    private static final double STRAFE_DISTANCE_IN = 15;
    private static final double SHOOT_VELOCITY = 1170;
    private static final double FEED_TIME_S = 0.3;
    private static final double FEED_TIME_3RD_S = 0.9;
    private static final double SPINUP_TIMEOUT_S = 2.5;
    private static final double INTAKE_TURN_DEGREES = 135;
    private static final double INTAKE_DRIVE_SPEED = 0.2;
    private static final double MAX_INTAKE_DISTANCE_IN = 80; // Search distance
    private static final double TURN_SPEED = 0.4;

    private ElapsedTime runtime = new ElapsedTime();
    private int intakeStartEncoder = 0;
    private double actualIntakeDistIn = 0;

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

        while (opModeIsActive() && currentState != State.DONE) {
            switch (currentState) {
                case DRIVE_FORWARD_1:
                    handleDrive(FORWARD_DISTANCE_IN, 0, State.SPIN_UP_SHOOTER);
                    break;

                case SPIN_UP_SHOOTER:
                    handleSpinUp(State.FEED_1ST, SPINUP_TIMEOUT_S);
                    break;

                case FEED_1ST:
                    handleFeed(0, 1.0, Separator.Direction.LEFT, FEED_TIME_S, State.RECOVER_2ND);
                    break;

                case RECOVER_2ND:
                    handleRecover(State.FEED_2ND);
                    break;

                case FEED_2ND:
                    handleFeed(1.0, 0, Separator.Direction.RIGHT, FEED_TIME_S, State.RECOVER_3RD);
                    break;

                case RECOVER_3RD:
                    handleRecover(State.FEED_3RD);
                    break;

                case FEED_3RD:
                    handleFeed(0, 1.0, Separator.Direction.LEFT, FEED_TIME_3RD_S, State.TURN_TO_INTAKE);
                    break;

                case TURN_TO_INTAKE:
                    handleTurn(INTAKE_TURN_DEGREES, State.INTAKE_BALLS);
                    if (currentState == State.INTAKE_BALLS) {
                        intakeStartEncoder = driveTrain.getEncoderPosition();
                    }
                    break;

                case INTAKE_BALLS:
                    intake.in(1.0);
                    // Automatic sorting
                    if (!colorSensor.isStagedLeft()) {
                        separator.sortLeft();
                        lift.setIndividualPower(0.0, 1.0);
                    } else if (!colorSensor.isStagedRight()) {
                        separator.sortRight();
                        lift.setIndividualPower(1.0, 0.0);
                    } else {
                        separator.stop();
                        lift.stop();
                    }

                    if (driveTrain.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        driveTrain.driveToPosition(MAX_INTAKE_DISTANCE_IN, 0, INTAKE_DRIVE_SPEED);
                    }

                    boolean collectedThree = colorSensor.isStagedLeft() && colorSensor.isStagedRight() && colorSensor.isStagedCenter();
                    if (collectedThree || !driveTrain.isBusy() || isStopRequested()) {
                        actualIntakeDistIn = (driveTrain.getEncoderPosition() - intakeStartEncoder) / OmniwheelDrive.COUNTS_PER_INCH;
                        driveTrain.stop();
                        intake.stop();
                        lift.stop();
                        separator.stop();
                        currentState = State.DRIVE_BACK_TO_SHOOT;
                    }
                    break;

                case DRIVE_BACK_TO_SHOOT:
                    handleDrive(-actualIntakeDistIn, 0, State.TURN_TO_SHOOT);
                    break;

                case TURN_TO_SHOOT:
                    handleTurn(0, State.SHOOT_COLLECTED_BALLS); // Turn back to 0 degrees
                    break;

                case SHOOT_COLLECTED_BALLS:
                    handleFeed(0, 1.0, Separator.Direction.LEFT, FEED_TIME_S, State.RECOVER_COLLECTED_2);
                    break;
                
                case RECOVER_COLLECTED_2:
                    handleRecover(State.SHOOT_COLLECTED_2);
                    break;
                
                case SHOOT_COLLECTED_2:
                    handleFeed(1.0, 0, Separator.Direction.RIGHT, FEED_TIME_S, State.RECOVER_COLLECTED_3);
                    break;
                
                case RECOVER_COLLECTED_3:
                    handleRecover(State.SHOOT_COLLECTED_3);
                    break;
                
                case SHOOT_COLLECTED_3:
                    // Final shot: run both lifts or just left
                    handleFeed(0, 1.0, Separator.Direction.LEFT, FEED_TIME_S, State.DRIVE_STRAFE_2);
                    break;

                case DRIVE_STRAFE_2:
                    handleDrive(0, STRAFE_DISTANCE_IN, State.DONE);
                    break;

                case INIT:
                case DONE:
                    break;
            }

            telemetry.addData("Current State", currentState.toString());
            telemetry.addData("Balls (Staged)", "L:%b R:%b C:%b", 
                    colorSensor.isStagedLeft(), colorSensor.isStagedRight(), colorSensor.isStagedCenter());
            telemetry.addData("Distances (CM)", "L:%.1f R:%.1f C:%.1f", 
                    colorSensor.getDistance(ColorSensorDetector.SensorLocation.LEFT, DistanceUnit.CM),
                    colorSensor.getDistance(ColorSensorDetector.SensorLocation.RIGHT, DistanceUnit.CM),
                    colorSensor.getDistance(ColorSensorDetector.SensorLocation.CENTER, DistanceUnit.CM));
            telemetry.addData("Heading", "%.2f", driveTrain.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        driveTrain.stop();
        shooter.stop();
        lift.stop();
        separator.stop();
        intake.stop();
    }

    private void handleDrive(double forwardInches, double strafeInches, State nextState) {
        if (driveTrain.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            driveTrain.driveToPosition(forwardInches, strafeInches, DRIVE_POWER);
        }
        if (!driveTrain.isBusy() || isStopRequested()) {
            driveTrain.stop();
            currentState = nextState;
        }
    }

    private void handleTurn(double degrees, State nextState) {
        driveTrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double error = degrees - driveTrain.getHeading(AngleUnit.DEGREES);
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;

        if (Math.abs(error) > 2) {
            driveTrain.drive(0, 0, Math.signum(error) * TURN_SPEED);
        } else {
            driveTrain.stop();
            currentState = nextState;
        }
    }

    private void handleSpinUp(State nextState, double timeout) {
        if (shooter.getVelocity() == 0) {
            shooter.setVelocity(SHOOT_VELOCITY);
            runtime.reset();
        }
        if (shooter.isAtTargetVelocity() || runtime.seconds() > timeout || isStopRequested()) {
            currentState = nextState;
            runtime.reset();
        }
    }

    private void handleFeed(double rightLiftPower, double leftLiftPower, Separator.Direction separatorDir, double duration, State nextState) {
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
            currentState = nextState;
            runtime.reset();
        }
    }

    private void handleRecover(State nextState) {
        if (shooter.isAtTargetVelocity() || isStopRequested()) {
            currentState = nextState;
            runtime.reset();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Basic OpMode to test Omniwheel/Mecanum movement.
 */
@TeleOp(name="MyTest: Omniwheel Movement", group="Test")
public class MyTest extends LinearOpMode {

    // Declare OpMode members for the motors.
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the names used here must correspond
        // to the names assigned during the robot configuration on the RC phone.
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Most robots need the motors to be reversed to drive forward.
        // The motor directions should be set based on your robot configuration.
        // Adjust these based on how your motors are mounted!
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake mode to stop them instantly
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get stick inputs
            double y = -gamepad1.left_stick_y; // Remember, Y is reversed on a gamepad
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            // Mecanum drive calculations (omnidirectional movement)
            double frontLeftPower = y + x + rotation;
            double frontRightPower = y - x - rotation;
            double backLeftPower = y - x + rotation;
            double backRightPower = y + x - rotation;

            // Normalize the values so none exceed 1.0 (or -1.0)
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Send power to the motors
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            // Show current telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Front Left Power", "%.2f", frontLeftPower);
            telemetry.addData("Front Right Power", "%.2f", frontRightPower);
            telemetry.addData("Back Left Power", "%.2f", backLeftPower);
            telemetry.addData("Back Right Power", "%.2f", backRightPower);
            telemetry.update();
        }
    }
}
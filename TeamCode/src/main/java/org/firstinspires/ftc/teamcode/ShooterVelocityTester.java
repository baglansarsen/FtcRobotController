package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Separator;

/**
 * Troubleshooting OpMode for the Shooter mechanism.
 * Allows real-time adjustment of target velocity and shows distance sensor data.
 */
@Disabled
@TeleOp(name = "Shooter Velocity Tester", group = "Test")
public class ShooterVelocityTester extends LinearOpMode {

    private Shooter shooter = new Shooter();
    private Lift lift = new Lift();
    private Separator separator = new Separator();
    private DistanceSensor sensor_distance;

    private double targetVelocity = 1200; // Starting default
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;

    @Override
    public void runOpMode() {
        shooter.init(hardwareMap);
        lift.init(hardwareMap);
        separator.init(hardwareMap);
        sensor_distance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD Up/Down: +/- 100 velocity");
        telemetry.addLine("  DPAD Right/Left: +/- 10 velocity");
        telemetry.addLine("  A: Run Shooter");
        telemetry.addLine("  B: Stop Shooter");
        telemetry.addLine("  X: Run Lift & Separator (Fire)");
        telemetry.addLine("  Y: Stop Lift & Separator");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust velocity with debouncing
            if (gamepad1.dpad_up && !dpadUpLast) targetVelocity += 100;
            if (gamepad1.dpad_down && !dpadDownLast) targetVelocity -= 100;
            if (gamepad1.dpad_right && !dpadRightLast) targetVelocity += 10;
            if (gamepad1.dpad_left && !dpadLeftLast) targetVelocity -= 10;

            dpadUpLast = gamepad1.dpad_up;
            dpadDownLast = gamepad1.dpad_down;
            dpadRightLast = gamepad1.dpad_right;
            dpadLeftLast = gamepad1.dpad_left;

            // Shooter Controls
            if (gamepad1.a) {
                shooter.setVelocity(targetVelocity);
            } else if (gamepad1.b) {
                shooter.stop();
            }

            // Feed/Shot Controls
            if (gamepad1.x) {
                lift.up();
                separator.sortLeft();
            } else if (gamepad1.y) {
                lift.stop();
                separator.stop();
            }

            // Telemetry Data
            double currentVelocity = shooter.getVelocity();
            double velocityError = targetVelocity - currentVelocity;
            double distanceCm = sensor_distance.getDistance(DistanceUnit.CM);

            telemetry.addData("--- TARGET ---", "%.2f", targetVelocity);
            telemetry.addData("--- ACTUAL ---", "%.2f", currentVelocity);
            telemetry.addData("--- ERROR  ---", "%.2f", velocityError);
            telemetry.addData("--- DISTANCE (cm) ---", "%.2f", distanceCm);
            telemetry.addData("Shooter Ready", shooter.isAtTargetVelocity());
            telemetry.addLine();
            telemetry.addData("Lift Power", "Running: " + gamepad1.x);
            telemetry.update();
        }

        shooter.stop();
        lift.stop();
        separator.stop();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.WebcamTroubleshooter;

/**
 * TeleOp OpMode for testing and troubleshooting the web camera and vision systems.
 * It uses the WebcamTroubleshooter mechanism to initialize the camera and display
 * diagnostic information, including AprilTag detections.
 */
@TeleOp(name = "Webcam Tester", group = "Main")
public class WebcamTester extends LinearOpMode {

    private WebcamTroubleshooter webcamTroubleshooter = new WebcamTroubleshooter();

    @Override
    public void runOpMode() {
        // Initialization
        telemetry.addData("Status", "Initializing Webcam...");
        telemetry.update();

        // The troubleshooter's init method takes the hardware map and telemetry
        webcamTroubleshooter.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Webcam Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Display all diagnostic information from the webcam system
            webcamTroubleshooter.telemetryOutput(telemetry);
            
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        // Clean up: Close the camera when the OpMode stops
        webcamTroubleshooter.stopCamera();
    }
}
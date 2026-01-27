package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * A mechanism class dedicated to initializing the web camera and AprilTag processor
 * primarily for troubleshooting and displaying all available vision data on telemetry.
 */
public class WebcamTroubleshooter {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    /**
     * Initializes the web camera and the AprilTag processor.
     * @param hwMap The HardwareMap from the OpMode.
     * @param telemetry The OpMode's telemetry object for immediate feedback.
     */
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
            .build();

        // Adjust the AprilTag detection settings
        aprilTag.setDecimation(2);

        // Create the Vision Portal, passing in the processor.
        visionPortal = new VisionPortal.Builder()
            .setCamera(hwMap.get(WebcamName.class, "Webcam 1")) // Assuming "Webcam 1" is the name
            .addProcessor(aprilTag)
            .build();

        telemetry.addData("Webcam Status", "VisionPortal Initialized");
        telemetry.update();
    }

    /**
     * Sends all available AprilTag and camera data to the telemetry.
     * Should be called repeatedly within the OpMode loop.
     * @param telemetry The OpMode's telemetry object.
     */
    public void telemetryOutput(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ (in):  %.1f, %.1f, %.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.y));
                telemetry.addLine(String.format("RBE (deg): %.1f, %.1f, %.1f", detection.ftcPose.roll, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center (px): %.0f, %.0f", detection.center.x, detection.center.y));
            }
        }
        
        telemetry.addData("Camera State", visionPortal.getCameraState());
    }

    /**
     * Getter for the AprilTagProcessor to allow OpModes to access detection data.
     */
    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    /**
     * Stops streaming to save processing power.
     */
    public void stopCamera() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // Optional: Add methods to manually start/stop streaming or select active processor
    public void setProcessor(boolean enableAprilTag) {
        if (enableAprilTag) {
            visionPortal.resumeStreaming();
        } else {
            visionPortal.stopStreaming();
        }
    }
}
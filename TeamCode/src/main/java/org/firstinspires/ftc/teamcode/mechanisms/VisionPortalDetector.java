package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * A mechanism class for managing a webcam and AprilTag detection using the VisionPortal.
 */
public class VisionPortalDetector {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    public VisionPortalDetector() {
        // Default constructor
    }

    /**
     * Initializes the webcam and the AprilTag processor.
     * @param hwMap The HardwareMap from the OpMode. Assumes the webcam is named "Webcam 1".
     */
    public void init(HardwareMap hwMap) {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
            // Optional: You can add these lines to draw debugging visuals on the Robot Controller screen.
            //.setDrawTagID(true)
            //.setDrawTagOutline(true)
            //.setDrawAxes(true)
            //.setDrawCubeProjection(true)
            .build();

        // Create the VisionPortal with the AprilTag processor.
        visionPortal = new VisionPortal.Builder()
            .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640, 480)) // Common resolution; adjust if needed.
            .addProcessor(aprilTag)
            .build();
    }

    /**
     * Gets the list of all current AprilTag detections.
     * @return A list of {@link AprilTagDetection} objects. The list will be empty if no tags are visible.
     */
    public List<AprilTagDetection> getDetections() {
        if (visionPortal == null) {
            return null;
        }
        return aprilTag.getDetections();
    }

    /**
     * Stops the vision portal's streaming to save CPU and battery.
     * Call this when you are not actively using vision.
     */
    public void stopStreaming() {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopStreaming();
        }
    }

    /**
     * Resumes the vision portal's streaming.
     */
    public void resumeStreaming() {
        if (visionPortal != null && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            visionPortal.resumeStreaming();
        }
    }

    /**
     * Closes the VisionPortal and releases all associated resources.
     * It's important to call this at the end of your OpMode to prevent errors.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

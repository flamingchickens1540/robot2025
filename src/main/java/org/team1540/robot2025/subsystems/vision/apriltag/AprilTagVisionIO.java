package org.team1540.robot2025.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public abstract class AprilTagVisionIO {
    public final String name;

    public AprilTagVisionIO(String name) {
        this.name = name;
    }

    @AutoLog
    public static class AprilTagVisionIOInputs {
        public boolean connected = false;
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] seenTagIDs = new int[0];
    }

    public record PoseObservation(
            Pose3d estimatedPoseMeters,
            int numTagsSeen,
            double avgTagDistance,
            double lastMeasurementTimestampSecs,
            double ambiguity) {}

    public void updateInputs(AprilTagVisionIOInputs inputs) {}
}

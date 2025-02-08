package org.team1540.robot2025.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
    @AutoLog
    class AprilTagVisionIOInputs {
        public boolean connected = false;
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    record PoseObservation(
            Pose3d estimatedPoseMeters,
            int numTagsSeen,
            double avgTagDistance,
            double lastMeasurementTimestampSecs,
            int[] seenTagIDs,
            double ambiguity) {}

    default void updateInputs(AprilTagVisionIOInputs inputs) {}

    default String getName() {
        return "";
    }
}

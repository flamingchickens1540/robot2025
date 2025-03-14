package org.team1540.robot2025.subsystems.vision.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CoralVisionIO {
    @AutoLog
    class CoralVisionIOInputs {
        public boolean connected = false;
        public boolean hasDetection = false;
        public CoralDetection latestDetection = new CoralDetection(0.0, new Rotation2d(), new Rotation2d(), 0.0);
    }

    record CoralDetection(double lastDetectionTimestampSecs, Rotation2d tx, Rotation2d ty, double targetArea) {}

    default void updateInputs(CoralVisionIOInputs inputs) {}

    default String getName() {
        return "";
    }
}

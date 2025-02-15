package org.team1540.robot2025.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
        public double yawVelocityRadPerSec = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}
}

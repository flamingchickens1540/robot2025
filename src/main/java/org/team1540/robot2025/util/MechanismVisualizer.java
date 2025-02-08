package org.team1540.robot2025.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

public class MechanismVisualizer {
    private static Pose3d arm = new Pose3d();
    private static Pose3d elevator = new Pose3d();

    public static void periodic() {
        Logger.recordOutput("Mechanisms", arm, elevator);
    }

    public static void setElevatorPosition(double positionMeters) {
        elevator = new Pose3d(0.0, 0.0, positionMeters, new Rotation3d());
    }

    public static void setArmRotation(Rotation2d rotation) {
        arm = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0, rotation.getRadians(), 0));
    }
}

package org.team1540.robot2025.subsystems.vision.coral;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class CoralVisionConstants {
    public static final int PIPELINE_INDEX = 0;
    public static final String CAMERA_NAME = "coral-camera";
    public static final Pose3d CAMERA_POSE =
            //            new Pose3d(
            //            Units.inchesToMeters(0),
            //            Units.inchesToMeters(0),
            //            Units.inchesToMeters(40),
            //            new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(-10)));
            new Pose3d(
                    Units.inchesToMeters(11.496868),
                    Units.inchesToMeters(8.546354),
                    Units.inchesToMeters(40.470175),
                    new Rotation3d(Math.toRadians(0), Math.toRadians(-41.865713), Math.toRadians(-6.274915)));
}

package org.team1540.robot2025.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagVisionConstants {
    public static final String FL_CAMERA_NAME = "front-left-camera";
    public static final String FR_CAMERA_NAME = "front-right-camera";
    public static final String BL_CAMERA_NAME = "back-left-camera";
    public static final String BR_CAMERA_NAME = "back-right-camera";

    // TODO: get camera transforms
    public static final Transform3d FL_CAMERA_TRANSFORM = new Transform3d();
    public static final Transform3d FR_CAMERA_TRANSFORM = new Transform3d();
    public static final Transform3d BL_CAMERA_TRANSFORM = new Transform3d();
    public static final Transform3d BR_CAMERA_TRANSFORM = new Transform3d();

    public static final double XY_STD_DEV_COEFF = 0.1;
    public static final double ROT_STD_DEV_COEFF = 0.5;

    public static final double MIN_ACCEPTED_NUM_TAGS = 1;
    public static final double MAX_OUTSIDE_OF_FIELD_TOLERANCE = 1;
    public static final double MAX_ROBOT_Z_TOLERANCE = 1;

    public static final int SIM_RES_WIDTH = 1280;
    public static final int SIM_RES_HEIGHT = 800;
    public static final Rotation2d SIM_DIAGONAL_FOV = Rotation2d.fromDegrees(70);
    public static final double SIM_FPS = 25.0;
    public static final double SIM_AVG_LATENCY_MS = 12.5;
}

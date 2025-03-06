package org.team1540.robot2025.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AprilTagVisionConstants {
    public static final String FL_CAMERA_NAME = "front-left";
    public static final String FR_CAMERA_NAME = "front-right";
    public static final String BL_CAMERA_NAME = "back-left";
    public static final String BR_CAMERA_NAME = "back-right";

    public static final Transform3d FL_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(11.933),
            Units.inchesToMeters(12.329),
            Units.inchesToMeters(8.281),
            new Rotation3d(0.0, Math.toRadians(-19), Math.toRadians(-10)));
    public static final Transform3d FR_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(11.933),
            Units.inchesToMeters(-12.329),
            Units.inchesToMeters(8.281),
            new Rotation3d(0.0, Math.toRadians(-19), Math.toRadians(10)));
    public static final Transform3d BL_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(-11.933),
            Units.inchesToMeters(12.329),
            Units.inchesToMeters(8.281),
            new Rotation3d(0.0, Math.toRadians(-19), Math.toRadians(180 + 26)));
    public static final Transform3d BR_CAMERA_TRANSFORM = new Transform3d(
            Units.inchesToMeters(-11.933),
            Units.inchesToMeters(-12.329),
            Units.inchesToMeters(8.281),
            new Rotation3d(0.0, Math.toRadians(-19), Math.toRadians(-180 - 26)));

    public static final double XY_STD_DEV_COEFF = 0.15;
    public static final double ROT_STD_DEV_COEFF = 0.25;

    public static final double MIN_ACCEPTED_NUM_TAGS = 1;
    public static final double MAX_OUTSIDE_OF_FIELD_TOLERANCE = 1;
    public static final double MAX_ROBOT_Z_TOLERANCE = 0.5;

    public static final int SIM_RES_WIDTH = 1280;
    public static final int SIM_RES_HEIGHT = 800;
    public static final Rotation2d SIM_DIAGONAL_FOV = Rotation2d.fromDegrees(70);
    public static final double SIM_FPS = 25.0;
    public static final double SIM_AVG_LATENCY_MS = 12.5;
}

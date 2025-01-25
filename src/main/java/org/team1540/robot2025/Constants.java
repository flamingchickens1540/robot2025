package org.team1540.robot2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private static final Mode kSimMode = Mode.SIM;
    public static final Mode kCurrentMode = Robot.isReal() ? Mode.REAL : kSimMode;

    private static final boolean kTuningMode = true;

    public static boolean isTuningMode() {
        return !DriverStation.isFMSAttached() && kTuningMode;
    }

    public enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public static final double kLoopPeriodSecs = 0.02;

    public static final double kRobotMassKg = Units.lbsToKilograms(147);
    public static final double kRobotMOIKgM2 = 5.8;

    public static final double kBumperLengthXMeters = Units.inchesToMeters(35.0);
    public static final double kBumperLengthYMeters = Units.inchesToMeters(33.0);

    public static class Vision {
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

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

        public static final double MAX_ACCEPTED_ROT_SPEED_RAD_PER_SEC = 1.0;
        public static final double MAX_ACCEPTED_LINEAR_SPEED_MPS = 4.0;
        public static final double MIN_ACCEPTED_NUM_TAGS = 1;
        public static final double MAX_ACCEPTED_AVG_TAG_DIST_METERS = 8.0;
        public static final double MAX_OUTSIDE_OF_FIELD_TOLERANCE = 1;
        public static final double MAX_ROBOT_Z_TOLERANCE = 1;

        public static final int SIM_RES_WIDTH = 1280;
        public static final int SIM_RES_HEIGHT = 960;
        public static final Rotation2d SIM_DIAGONAL_FOV = Rotation2d.fromDegrees(70);
        public static final double SIM_FPS = 14.5;
        public static final double SIM_AVG_LATENCY_MS = 100;
    }
}

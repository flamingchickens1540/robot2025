package org.team1540.robot2025;

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
    private static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;

    private static final boolean tuningMode = true;

    public static boolean isTuningMode() {
        return !DriverStation.isFMSAttached() && tuningMode;
    }

    public enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public static class Elevator {
        //TODO: Set these constants
        public static final int LEADER_ID = -1;
        public static final int FOLLOWER_ID = -1;

        public static final double SUPPLY_CURRENT_LIMIT = 70.0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
        public static final double SUPPLY_TIME_THRESHOLD = 0.5;

        public static final double GEAR_RATIO = 0.0;
        public static final double KP = 0.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;
        public static final double KG = 0.0;
    }
}

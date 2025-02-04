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

    public static class IntakeConstants {

        // TODO: tune

        public static final int TOP_FALCON_ID = 0;
        public static final int BOTTOM_FALCON_ID = 1;
        public static final int NEO_ID = 2;

        public static final double GRABBER_KS = 0.25;
        public static final double GRABBER_KV = 0.12;
        public static final double GRABBER_KA = 0.01;
        public static final double GRABBER_KP = 2.4;
        public static final double GRABBER_KI = 0;
        public static final double GRABBER_KD = 0.1;

        public static final int GRABBER_CRUISE_VELOCITY = 80;
        public static final int GRABBER_ACCELERATION = 160;
    }
}

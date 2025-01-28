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

    public static final double LOOP_PERIOD_SECS = 0.02;

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
}

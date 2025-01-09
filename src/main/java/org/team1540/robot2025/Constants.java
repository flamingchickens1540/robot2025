package org.team1540.robot2025;

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
}

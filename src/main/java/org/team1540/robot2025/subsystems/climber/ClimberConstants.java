package org.team1540.robot2025.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ClimberConstants {
    public static final double GEAR_RATIO = 125.0 * 54.0 / 12.0;
    public static final double ARM_MOMENT_OF_INERTIA_KGM2 = 0.02;
    public static final double ARM_LENGTH_METERS = 0.411; // TODO: check please

    public static final Translation3d ROTATIONAL_ORIGIN = new Translation3d(0.0, 0.173165, 0.179950);

    // arm parallel to floor is 0, min angle is 30
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(90);

    // PID loop constants
    public static final double KP = 25;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double KS = 0.03;
    public static final double KG = 0.41;
    public static final double KV = 0.55;

    public static final int MOTOR_ID = 2;

    public static final double CRUISE_VELOCITY_RPS = 3;
    public static final double MAX_ACCEL_RPS2 = 10;
    public static final double JERK_RPS = 2000;

    public static final Rotation2d ERROR_TOLERANCE = Rotation2d.fromDegrees(1);
}

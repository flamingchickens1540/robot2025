package org.team1540.robot2025.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
    public static final double GEAR_RATIO = 28;
    public static final double ARM_MOMENT_OF_INERTIA_KGM2 = 0.5;
    public static final double ARM_LENGTH_METERS = 0.411; // TODO: check please

    // arm parallel to floor is 0, min angle is 30
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-60);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(240);

    // PID loop constants
    // TODO: what numbers are reasonable?
    public static final double KP = 300;
    public static final double KI = 50;
    public static final double KD = 1;

    // TODO: what values are reasonable?
    public static final double MAX_VELOCITY_RPS = 30;
    public static final double MAX_ACCELERATION = 3;

    // TODO: get reasonable values
    // TODO: try to understand
    public static final double KS = 0.03;
    public static final double KG = 0;
    public static final double KV = 0.8;

    public static final int CANCODER_ID = 0; // TODO: get id
    public static final int MOTOR_ID = 0;
    public static final double CANCODER_TO_PIVOT_RATIO = 1;
    public static final double MOTOR_TO_CANCODER = 1;
    public static final double CANCODER_OFFSET_ROTS = 0.2; // TODO: get offset
    public static final double DISCONTINUITY_POINT = 1; // makes the range 0-1

    public static final double CRUISE_VELOCITY_RPS = 3;
    public static final double MAX_ACCEL_RPS2 = 15;
    public static final double JERK_RPS = 2000; // TODO: uhhh random numbers yay!

    public static final Rotation2d ERROR_TOLERANCE = Rotation2d.fromDegrees(0.7);
}

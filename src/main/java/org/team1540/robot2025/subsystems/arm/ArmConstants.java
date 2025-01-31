package org.team1540.robot2025.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
    public static final double GEAR_RATIO = 28;
    public static final double ARM_MOMENT_OF_INERTIA = 0; // TODO:
    public static final double ARM_LENGTH_METERS = 0.411; // TODO: check please

    // arm straight down is 0, min angle is 30
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(30);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(330);
    public static final Rotation2d STARTING_ANGLE = MIN_ANGLE;
    public static final double MEASUREMENT_STANDARD_DEVIATIONS = 0; // TODO:

    // PID loop constants
    // TODO: what numbers are reasonable?
    public static final double KP = 300;
    public static final double KI = 50;
    public static final double KD = 1;

    // TODO: what values are reasonable?
    public static final double MAX_VELOCITY = 30;
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

    public static final double CRUISE_VELOCITY_RPS = 1;
    public static final double MAX_ACCEL_RPS = 2;
    public static final double JERK_RPS = 2000; // TODO: uhhh random numbers yay!

    public static final double CURRENT_LIMIT = 10;
    public static final double CURRENT_LOWER_LIMIT = 0.1;
    public static final double TIME_LOWER_LIMIT = 15;
}

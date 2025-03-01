package org.team1540.robot2025.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ArmConstants {
    public static final double GEAR_RATIO = 28;
    public static final double ARM_MOMENT_OF_INERTIA_KGM2 = 0.02;
    public static final double ARM_LENGTH_METERS = 0.411; // TODO: check please

    public static final Translation3d ROTATIONAL_ORIGIN = new Translation3d(0.0, 0.173165, 0.179950);

    // arm parallel to floor is 0, min angle is 30
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-60);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(250);

    // PID loop constants
    public static final double KP = 80;
    public static final double KI = 0;
    public static final double KD = 5.0;

    public static final double KS = 0.03;
    public static final double KG = 0.43;
    public static final double KV = 0.3;

    public static final int CANCODER_ID = 30;
    public static final int MOTOR_ID = 15;
    public static final double CANCODER_TO_PIVOT_RATIO = 1;
    public static final double MOTOR_TO_CANCODER = 28;
    public static final double CANCODER_OFFSET_ROTS = -0.2080078125; // TODO: get offset
    public static final double DISCONTINUITY_POINT = 0.75;

    public static final double CRUISE_VELOCITY_RPS = 2.0;
    public static final double MAX_ACCEL_RPS2 = 4;
    public static final double JERK_RPS = 50;

    public static final Rotation2d ERROR_TOLERANCE = Rotation2d.fromDegrees(3);
}

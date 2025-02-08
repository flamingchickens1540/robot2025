package org.team1540.robot2025.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class CoralIntakeConstants {
    public static final int SPIN_MOTOR_ID = 0;
    public static final int PIVOT_MOTOR_ID = 1;
    public static final int FUNNEL_MOTOR_ID = 2;

    public static final double SPIN_GEAR_RATIO = 2.0;
    public static final double FUNNEL_GEAR_RATIO = 1.0;

    public static final double PIVOT_GEAR_RATIO = 9.0 * (48.0 / 28.0) * (36.0 / 18.0);
    public static final double PIVOT_KS = 0.01;
    public static final double PIVOT_KV = 0.58;
    public static final double PIVOT_KG = 0.2;
    public static final double PIVOT_KP = 40;
    public static final double PIVOT_KI = 0;
    public static final double PIVOT_KD = 0;

    public static final double PIVOT_CRUISE_VELOCITY_RPS = 2.5;
    public static final double PIVOT_ACCELERATION_RPS2 = 10.0;

    public static final Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(-8);
    public static final Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(90);

    public static final Translation3d ROTATIONAL_ORIGIN = new Translation3d(0.304800, 0.0, 0.1270000);

    public enum CoralIntakeState {
        STOW(PIVOT_MAX_ANGLE, 0.0, 0.0),
        INTAKE(PIVOT_MIN_ANGLE, 12.0, 12.0),
        EJECT(PIVOT_MIN_ANGLE, -12.0, -12.0);

        public final Rotation2d pivotPosition;
        public final double rollerVoltage;
        public final double funnelVoltage;

        CoralIntakeState(Rotation2d pivotPosition, double rollerVoltage, double funnelVoltage) {
            this.pivotPosition = pivotPosition;
            this.rollerVoltage = rollerVoltage;
            this.funnelVoltage = funnelVoltage;
        }
    }
}

package org.team1540.robot2025.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    // TODO: Set these constants
    public static final int LEADER_ID = -1;
    public static final int FOLLOWER_ID = -1;
    public static final double SUPPLY_CURRENT_LIMIT = 70.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
    public static final double SUPPLY_TIME_THRESHOLD = 0.5;
    public static final double MIN_HEIGHT = 0.0;
    public static final double MAX_HEIGHT = 2.0;

    public static final double GEAR_RATIO = 0.0;
    public static final double KP = 0.5;
    public static final double KI = 0.1;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    public static final double CRUISE_VELOCITY_MPS = 1.2;
    public static final double MAXIMUM_ACCELERATION_MPS2 = 50;
    public static final double JERK_MPS3 = 200;
    public static final double SPROCKET_RADIUS_M = Units.inchesToMeters(1.751 / 2);
    public static final double SPROCKET_CIRCUMFERENCE_M = 2 * SPROCKET_RADIUS_M * Math.PI;
    public static final double MOTOR_ROTS_PER_METER = GEAR_RATIO / SPROCKET_CIRCUMFERENCE_M;
    public static final double SIM_CARRIAGE_MASS_KG = 1.55; // TODO: check this number :)

    public enum ElevatorState {
        BASE(MIN_HEIGHT),
        L1(0.5),
        L2(1.0),
        L3(1.4),
        L4(1.8),
        BARGE(2.0);

        public final double elevatorHeight;

        ElevatorState(double height) {
            this.elevatorHeight = height;
        }
    }
}

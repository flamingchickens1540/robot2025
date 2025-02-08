package org.team1540.robot2025.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    // TODO: Fix these constants
    public static final int LEADER_ID = -1;
    public static final int FOLLOWER_ID = -1;
    public static final int UPPER_LIMIT_ID = -1;
    public static final int LOWER_LIMIT_ID = -1;
    public static final double MIN_HEIGHT_M = 0.0;
    public static final double MAX_HEIGHT_M = Units.inchesToMeters(63.25);
    public static final double STAGE_1_HEIGHT_M = Units.inchesToMeters(30.331);
    public static final double CLEAR_HEIGHT_M = 0.25;
    public static final double POS_ERR_TOLERANCE_M = 0.01;

    public static final double GEAR_RATIO = 3.0;
    public static final double KS = 0.03178;
    public static final double KV = 2.562;
    public static final double KA = 0.00;
    public static final double KP = 300;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KG = 0;

    public static final double CRUISE_VELOCITY_MPS = 3.5;
    public static final double MAXIMUM_ACCELERATION_MPS2 = 15.40;
    public static final double JERK_MPS3 = 200;
    public static final double SPROCKET_RADIUS_M = Units.inchesToMeters(1.751 / 2);
    public static final double SPROCKET_CIRCUMFERENCE_M = 2 * SPROCKET_RADIUS_M * Math.PI;
    public static final double MOTOR_ROTS_PER_METER = GEAR_RATIO / SPROCKET_CIRCUMFERENCE_M;
    public static final double SIM_CARRIAGE_MASS_KG = 1.55;

    public enum ElevatorState {
        BASE(MIN_HEIGHT_M),
        SOURCE(0.25),
        L1(0.5),
        L2(1.0),
        L3(1.2),
        L4(MAX_HEIGHT_M),
        BARGE(MAX_HEIGHT_M);

        public final double elevatorHeight;

        ElevatorState(double height) {
            this.elevatorHeight = height;
        }
    }
}

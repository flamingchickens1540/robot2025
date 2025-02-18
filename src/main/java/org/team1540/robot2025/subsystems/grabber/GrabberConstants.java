package org.team1540.robot2025.subsystems.grabber;

import edu.wpi.first.math.util.Units;

public class GrabberConstants {
    // TODO: Fix this
    public static final int MOTOR_ID = 13;
    public static final int CANDI_ID = 1;
    public static final int BEFORE_LASER_CAN_ID = 2;
    public static final int AFTER_LASER_CAN_ID = 3;
    public static final double LASER_CAN_DETECT_DISTANCE_MM = 10.0;

    public static final double MOI = 0.025;
    public static final double GEAR_RATIO = 24.0 / 36.0;

    public static final double Y_OFFSET_METERS = Units.inchesToMeters(-2.900);
}

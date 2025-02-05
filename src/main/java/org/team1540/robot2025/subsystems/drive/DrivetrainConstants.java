package org.team1540.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.generated.TunerConstants;

public class DrivetrainConstants {
    public static final double ODOMETRY_FREQUENCY = 250.0;
    public static final String CAN_BUS = TunerConstants.kCANBus.getName();

    public static final double DRIVEBASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    public static final double MAX_TOTAL_MODULE_FORCES = 4
            * DCMotor.getKrakenX60Foc(1)
                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio)
                    .getTorque(TunerConstants.FrontLeft.SlipCurrent)
            / TunerConstants.FrontLeft.WheelRadius;

    public static final double MAX_LINEAR_SPEED_MPS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_LINEAR_ACCELERATION = MAX_TOTAL_MODULE_FORCES / Constants.ROBOT_MASS_KG;

    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_LINEAR_SPEED_MPS / DRIVEBASE_RADIUS;
    public static final double MAX_ANGULAR_ACCEL_RAD_PER_SEC2 =
            MAX_TOTAL_MODULE_FORCES * DRIVEBASE_RADIUS / Constants.ROBOT_MOI_KGM2;
    public static final double MAX_STEER_SPEED_RAD_PER_SEC =
            DCMotor.getFalcon500Foc(1).withReduction(TunerConstants.FrontLeft.SteerMotorGearRatio).freeSpeedRadPerSec;

    public static final double WHEEL_COF = 1.4;

    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
            Constants.ROBOT_MASS_KG,
            Constants.ROBOT_MOI_KGM2,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    MAX_LINEAR_SPEED_MPS,
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            getModuleTranslations());

    public static final boolean OPTIMIZE_SETPOINTS = false;

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY),
        };
    }
}

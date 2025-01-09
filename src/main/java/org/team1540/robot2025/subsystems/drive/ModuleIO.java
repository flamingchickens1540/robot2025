package org.team1540.robot2025.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRads = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
        public Rotation2d turnPosition = Rotation2d.kZero;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelsius = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRads = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop value. */
    default void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    default void setTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    default void setDriveVelocity(double velocityRadPerSec) {}

    /** Run the turn motor to the specified rotation. */
    default void setTurnPosition(Rotation2d rotation) {}

    /** Sets the neutral mode of the drive motor */
    default void setDriveBrakeMode(boolean enabled) {}

    /** Set the neutral mode of the turn motor */
    default void setTurnBrakeMode(boolean enabled) {}
}

package org.team1540.robot2025.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
    public enum MountPosition {
        FL(0),
        FR(1),
        BL(2),
        BR(3);

        public final int index;

        MountPosition(int index) {
            this.index = index;
        }
    }

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final MountPosition mountPosition;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(
            ModuleIO io,
            MountPosition mountPosition,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.io = io;
        this.mountPosition = mountPosition;
        this.constants = constants;
        driveDisconnectedAlert =
                new Alert("Disconnected drive motor on " + mountPosition + " module.", AlertType.kError);
        turnDisconnectedAlert =
                new Alert("Disconnected steer motor on " + mountPosition + " module.", AlertType.kError);
        turnEncoderDisconnectedAlert =
                new Alert("Disconnected steer encoder on " + mountPosition + " module.", AlertType.kWarning);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain/" + mountPosition + "Module", inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRads[i] * constants.WheelRadius;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getTurnAngle());
        state.cosineScale(inputs.turnPosition);

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        io.setTurnPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(Rotation2d.kZero);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getTurnAngle() {
        return inputs.turnPosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getDrivePositionMeters() {
        return inputs.drivePositionRads * constants.WheelRadius;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getDriveVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * constants.WheelRadius;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getTurnAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getTurnAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRads;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }

    /** Sets the neutral mode of all motors. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
    }
}

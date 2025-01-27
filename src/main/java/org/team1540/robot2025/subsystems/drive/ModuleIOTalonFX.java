package org.team1540.robot2025.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.team1540.robot2025.generated.TunerConstants;
import org.team1540.robot2025.util.swerve.ModuleHW;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    // Hardware objects
    private final TalonFX drive;
    private final TalonFX turn;
    private final CANcoder cancoder;

    // Drive motor control requests
    private final VoltageOut driveVoltageReq = new VoltageOut(0);
    private final VelocityVoltage driveVelocityVoltageReq = new VelocityVoltage(0.0);
    private final TorqueCurrentFOC driveTorqueReq = new TorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC driveVelocityTorqueReq = new VelocityTorqueCurrentFOC(0.0);

    // Turn motor control requests
    private final VoltageOut turnVoltageReq = new VoltageOut(0);
    private final TorqueCurrentFOC turnTorqueReq = new TorqueCurrentFOC(0);
    private final PositionVoltage turnPositionVoltageReq = new PositionVoltage(0.0);
    private final PositionTorqueCurrentFOC turnPositionTorqueReq = new PositionTorqueCurrentFOC(0.0);

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampQueue;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;
    private final StatusSignal<Temperature> driveTemp;

    // Inputs from turn motor
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;
    private final StatusSignal<Temperature> turnTemp;

    private final Executor brakeModeExecutor = Executors.newFixedThreadPool(4);

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOTalonFX(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        ModuleHW hw = ModuleHW.fromModuleConstants(constants, TunerConstants.kCANBus.getName());
        drive = hw.driveMotor();
        turn = hw.turnMotor();
        cancoder = hw.turnEncoder();

        // Create timestamp queue
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();

        // Create drive status signals
        drivePosition = drive.getPosition();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(drive.getPosition());
        driveVelocity = drive.getVelocity();
        driveAppliedVolts = drive.getMotorVoltage();
        driveCurrent = drive.getStatorCurrent();
        driveTemp = drive.getDeviceTemp();

        // Create turn status signals
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turn.getPosition();
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turn.getPosition());
        turnVelocity = turn.getVelocity();
        turnAppliedVolts = turn.getMotorVoltage();
        turnCurrent = turn.getStatorCurrent();
        turnTemp = turn.getDeviceTemp();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(Drivetrain.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                driveTemp,
                turnAbsolutePosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent,
                turnTemp);
        ParentDevice.optimizeBusUtilizationForAll(drive, turn);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        StatusCode driveStatus =
                BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, driveTemp);
        StatusCode turnStatus =
                BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent, driveTemp);
        StatusCode turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
        inputs.driveTempCelsius = driveTemp.getValueAsDouble();

        // Update turn inputs
        inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
        inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
        inputs.turnTempCelsius = turnTemp.getValueAsDouble();

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble(value -> value).toArray();
        inputs.odometryDrivePositionsRads = drivePositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians)
                .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double input) {
        drive.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> driveVoltageReq.withOutput(input);
                    case TorqueCurrentFOC -> driveTorqueReq.withOutput(input);
                });
    }

    @Override
    public void setTurnOpenLoop(double input) {
        turn.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> turnVoltageReq.withOutput(input);
                    case TorqueCurrentFOC -> turnTorqueReq.withOutput(input);
                });
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        drive.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> driveVelocityVoltageReq.withVelocity(velocityRotPerSec);
                    case TorqueCurrentFOC -> driveVelocityTorqueReq.withVelocity(velocityRotPerSec);
                });
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turn.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> turnPositionVoltageReq.withPosition(rotation.getRotations());
                    case TorqueCurrentFOC -> turnPositionTorqueReq.withPosition(rotation.getRotations());
                });
    }

    @Override
    public void setDriveBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            synchronized (drive) {
                MotorOutputConfigs config = new MotorOutputConfigs();
                drive.getConfigurator().refresh(config);
                config.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                drive.getConfigurator().apply(config);
            }
        });
    }

    @Override
    public void setTurnBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            synchronized (turn) {
                MotorOutputConfigs config = new MotorOutputConfigs();
                turn.getConfigurator().refresh(config);
                config.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                turn.getConfigurator().apply(config);
            }
        });
    }
}

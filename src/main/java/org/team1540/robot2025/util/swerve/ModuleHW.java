package org.team1540.robot2025.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.team1540.robot2025.util.PhoenixUtil;

public record ModuleHW(TalonFX driveMotor, TalonFX turnMotor, CANcoder turnEncoder) {
    public static ModuleHW fromModuleConstants(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
            String canBus) {
        TalonFX drive = new TalonFX(constants.DriveMotorId, canBus);
        TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;

        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 70;
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Slot0 = constants.DriveMotorGains;

        PhoenixUtil.tryUntilOk(5, () -> drive.getConfigurator().apply(driveConfig));

        CANcoder turnEncoder = new CANcoder(constants.EncoderId, canBus);
        CANcoderConfiguration turnEncoderConfig = constants.EncoderInitialConfigs;
        turnEncoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        turnEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turnEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        PhoenixUtil.tryUntilOk(5, () -> turnEncoder.getConfigurator().apply(turnEncoderConfig));

        TalonFX turn = new TalonFX(constants.SteerMotorId, canBus);
        TalonFXConfiguration turnConfig = constants.SteerMotorInitialConfigs;

        turnConfig.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Feedback.FeedbackSensorSource = switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
            case FusedCANdiPWM1 -> FeedbackSensorSourceValue.FusedCANdiPWM1;
            case FusedCANdiPWM2 -> FeedbackSensorSourceValue.FusedCANdiPWM2;
            case SyncCANdiPWM1 -> FeedbackSensorSourceValue.SyncCANdiPWM1;
            case SyncCANdiPWM2 -> FeedbackSensorSourceValue.SyncCANdiPWM2;
            case RemoteCANdiPWM1 -> FeedbackSensorSourceValue.RemoteCANdiPWM1;
            case RemoteCANdiPWM2 -> FeedbackSensorSourceValue.RemoteCANdiPWM2;
            default -> throw new IllegalStateException(
                    "Invalid turn feedback sensor source" + constants.FeedbackSource);};

        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.Feedback.SensorToMechanismRatio = 1.0;
        turnConfig.CurrentLimits.SupplyCurrentLimit = 70;
        turnConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        turnConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.Slot0 = constants.SteerMotorGains;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity =
                Units.radiansToRotations(DCMotor.getFalcon500Foc(1).freeSpeedRadPerSec) / constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.1;
        turnConfig.MotionMagic.MotionMagicExpo_kV = constants.SteerMotorGearRatio
                * Units.radiansToRotations(DCMotor.getFalcon500Foc(1).freeSpeedRadPerSec)
                / 12.0;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        PhoenixUtil.tryUntilOk(5, () -> turn.getConfigurator().apply(turnConfig));

        return new ModuleHW(drive, turn, turnEncoder);
    }
}

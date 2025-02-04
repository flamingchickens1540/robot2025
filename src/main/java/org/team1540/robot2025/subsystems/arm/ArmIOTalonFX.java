package org.team1540.robot2025.subsystems.arm;

import static org.team1540.robot2025.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX motor = new TalonFX(MOTOR_ID);
    private final CANcoder cancoder = new CANcoder(CANCODER_ID);

    private final StatusSignal<Angle> motorPosition = motor.getPosition();
    private final StatusSignal<Angle> cancoderPosition = cancoder.getPosition();
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVoltage = motor.getMotorVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> temp = motor.getDeviceTemp();
    private final MotionMagicVoltage positionCtrlReq = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageCtrlReq = new VoltageOut(0);

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // constructor
    public ArmIOTalonFX() {

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: triple check inverted

        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = CANCODER_ID;
        motorConfig.Feedback.SensorToMechanismRatio = CANCODER_TO_PIVOT_RATIO * MOTOR_TO_CANCODER;
        motorConfig.Feedback.RotorToSensorRatio = MOTOR_TO_CANCODER;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.getRotations();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MIN_ANGLE.getRotations();

        motorConfig.Slot0.kP = KP;
        motorConfig.Slot0.kI = KI;
        motorConfig.Slot0.kD = KD;
        motorConfig.Slot0.kS = KS;
        motorConfig.Slot0.kG = KG;
        motorConfig.Slot0.kV = KV;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        /*
        Note that the sensor offset and ratios must be
        configured so that the sensor reports a position
         of 0 when the mechanism is horizonal
         (parallel to the ground), and the reported
         sensor position is 1:1 with the mechanism.
         */

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCEL_RPS2;
        motorConfig.MotionMagic.MotionMagicJerk = JERK_RPS;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = CURRENT_LOWER_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = TIME_LOWER_LIMIT;

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = CANCODER_OFFSET_ROTS;
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = DISCONTINUITY_POINT;

        cancoder.getConfigurator().apply(cancoderConfig);
        motor.getConfigurator().apply(motorConfig);
        motor.setPosition(motorPosition.getValueAsDouble() * MOTOR_TO_CANCODER);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                motorPosition,
                cancoderPosition,
                velocity,
                appliedVoltage,
                supplyCurrentAmps,
                statorCurrentAmps,
                temp);

        motor.optimizeBusUtilization();
        cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                motorPosition, cancoderPosition, velocity, appliedVoltage, supplyCurrentAmps, statorCurrentAmps, temp);
        // TODO: SOFT LIMITS! YAY
        inputs.position = Rotation2d.fromRotations(motorPosition.getValueAsDouble());
        inputs.velocityRPM = velocity.getValueAsDouble() * 60; // converting from rps to rpm
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void setMotorPosition(Rotation2d motorPosition) {
        motor.setControl(positionCtrlReq.withPosition(motorPosition.getRotations()));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageCtrlReq.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        motor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        Slot0Configs pidConfigs = motorConfig.Slot0;
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        motor.getConfigurator().apply(pidConfigs);
    }

    @Override
    public void configFeedForwardTerms(double kG, double kS, double kV) {
        Slot0Configs pidConfigs = motorConfig.Slot0;
        pidConfigs.kG = kG;
        pidConfigs.kS = kS;
        pidConfigs.kV = kV;
        motor.getConfigurator().apply(pidConfigs);
    }
}

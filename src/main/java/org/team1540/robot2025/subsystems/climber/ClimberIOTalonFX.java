package org.team1540.robot2025.subsystems.climber;

import static org.team1540.robot2025.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX motor = new TalonFX(MOTOR_ID);

    private final StatusSignal<Angle> motorPosition = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVoltage = motor.getMotorVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> temp = motor.getDeviceTemp();
    private final MotionMagicVoltage positionCtrlReq = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageCtrlReq = new VoltageOut(0);

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

    // constructor
    public ClimberIOTalonFX() {
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        motorConfig.Slot0.kP = KP;
        motorConfig.Slot0.kI = KI;
        motorConfig.Slot0.kD = KD;
        motorConfig.Slot0.kS = KS;
        motorConfig.Slot0.kG = KG;
        motorConfig.Slot0.kV = KV;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCEL_RPS2;
        motorConfig.MotionMagic.MotionMagicJerk = JERK_RPS;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 0.1;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 15;

        motor.getConfigurator().apply(motorConfig);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50, motorPosition, velocity, appliedVoltage, supplyCurrentAmps, statorCurrentAmps, temp);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        StatusCode motorStatus = BaseStatusSignal.refreshAll(
                motorPosition, velocity, appliedVoltage, supplyCurrentAmps, statorCurrentAmps, temp);

        inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());

        inputs.position = Rotation2d.fromRotations(motorPosition.getValueAsDouble());

        inputs.velocityRPM = velocity.getValueAsDouble() * 60; // converting from rps to rpm
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void setSetpoint(Rotation2d motorPosition) {
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
    public void configFF(double kS, double kV, double kG) {
        Slot0Configs pidConfigs = motorConfig.Slot0;
        pidConfigs.kG = kG;
        pidConfigs.kS = kS;
        pidConfigs.kV = kV;
        motor.getConfigurator().apply(pidConfigs);
    }
}

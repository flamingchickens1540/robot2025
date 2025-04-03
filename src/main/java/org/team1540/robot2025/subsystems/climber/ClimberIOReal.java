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
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class ClimberIOReal implements ClimberIO {
    private final TalonFX pivotMotor = new TalonFX(PIVOT_MOTOR_ID);

    private final StatusSignal<Angle> motorPosition = pivotMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocity = pivotMotor.getVelocity();
    private final StatusSignal<Voltage> appliedVoltage = pivotMotor.getMotorVoltage();
    private final StatusSignal<Current> statorCurrentAmps = pivotMotor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = pivotMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> temp = pivotMotor.getDeviceTemp();
    private final MotionMagicVoltage positionCtrlReq = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageCtrlReq = new VoltageOut(0);

    private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    private final Debouncer pivotConnectedDebounce = new Debouncer(0.5);

    private final SparkMax rollerMotor = new SparkMax(ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

    SparkMaxConfig rollerConfig = new SparkMaxConfig();

    private final Debouncer rollerConnectedDebounce = new Debouncer(0.5);

    // constructor
    public ClimberIOReal() {
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;

        pivotConfig.Slot0.kP = KP;
        pivotConfig.Slot0.kI = KI;
        pivotConfig.Slot0.kD = KD;
        pivotConfig.Slot0.kS = KS;
        pivotConfig.Slot0.kG = KG;
        pivotConfig.Slot0.kV = KV;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        pivotConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCEL_RPS2;
        pivotConfig.MotionMagic.MotionMagicJerk = JERK_RPS;

        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 50;
        pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 0.1;
        pivotConfig.CurrentLimits.SupplyCurrentLowerTime = 15;

        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(65);
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        pivotMotor.getConfigurator().apply(pivotConfig);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50, motorPosition, velocity, appliedVoltage, supplyCurrentAmps, statorCurrentAmps, temp);

        pivotMotor.optimizeBusUtilization();

        rollerConfig.smartCurrentLimit(20);
        rollerConfig.inverted(false);
        rollerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        rollerConfig.encoder.positionConversionFactor(1.0 / ROLLER_GEAR_RATIO);

        rollerMotor.configure(
                rollerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        StatusCode motorStatus = BaseStatusSignal.refreshAll(
                motorPosition, velocity, appliedVoltage, supplyCurrentAmps, statorCurrentAmps, temp);

        inputs.motorConnected = pivotConnectedDebounce.calculate(motorStatus.isOK());

        inputs.pivotPosition = Rotation2d.fromRotations(motorPosition.getValueAsDouble());

        inputs.pivotVelocityRPM = velocity.getValueAsDouble() * 60; // converting from rps to rpm
        inputs.pivotAppliedVolts = appliedVoltage.getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.pivotStatorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.pivotTempCelsius = temp.getValueAsDouble();

        inputs.rollerAppliedVolts = (rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage());
        inputs.rollerVelocityRPM = rollerEncoder.getVelocity() / 60.0;
        inputs.rollerSupplyCurrentAmps = rollerMotor.getOutputCurrent();
        inputs.rollerStatorCurrentAmps = rollerMotor.getOutputCurrent();
        inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerMotor.getLastError() == REVLibError.kOk);
    }

    @Override
    public void setPivotSetpoint(Rotation2d motorPosition) {
        pivotMotor.setControl(positionCtrlReq.withPosition(motorPosition.getRotations()));
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public void resetPivotPosition(Rotation2d position) {
        pivotMotor.setPosition(position.getRotations());
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setControl(voltageCtrlReq.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        pivotMotor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        Slot0Configs pidConfigs = pivotConfig.Slot0;
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        pivotMotor.getConfigurator().apply(pidConfigs);
    }

    @Override
    public void configFF(double kS, double kV, double kG) {
        Slot0Configs pidConfigs = pivotConfig.Slot0;
        pidConfigs.kG = kG;
        pidConfigs.kS = kS;
        pidConfigs.kV = kV;
        pivotMotor.getConfigurator().apply(pidConfigs);
    }
}

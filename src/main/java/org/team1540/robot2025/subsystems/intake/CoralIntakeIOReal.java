package org.team1540.robot2025.subsystems.intake;

import static org.team1540.robot2025.subsystems.intake.CoralIntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class CoralIntakeIOReal implements CoralIntakeIO {
    // rotation of horizontal beams for the intake, clockwise to intake, counter-clockwise to spit out
    private final TalonFX spinFalcon = new TalonFX(SPIN_MOTOR_ID);

    private final StatusSignal<AngularVelocity> spinVelocity = spinFalcon.getVelocity();
    private final StatusSignal<Angle> spinPosition = spinFalcon.getPosition();
    private final StatusSignal<Voltage> spinAppliedVoltage = spinFalcon.getMotorVoltage();
    private final StatusSignal<Current> spinSupplyCurrent = spinFalcon.getSupplyCurrent();
    private final StatusSignal<Current> spinStatorCurrent = spinFalcon.getStatorCurrent();
    private final StatusSignal<Temperature> spinTemp = spinFalcon.getDeviceTemp();

    private final VoltageOut spinVoltageRequest = new VoltageOut(0);

    // controls position of intake
    private final TalonFX pivotFalcon = new TalonFX(PIVOT_MOTOR_ID);

    private final StatusSignal<AngularVelocity> pivotVelocity = pivotFalcon.getVelocity();
    private final StatusSignal<Angle> pivotPosition = pivotFalcon.getPosition();
    private final StatusSignal<Voltage> pivotAppliedVoltage = pivotFalcon.getMotorVoltage();
    private final StatusSignal<Current> pivotSupplyCurrent = pivotFalcon.getSupplyCurrent();
    private final StatusSignal<Current> pivotStatorCurrent = pivotFalcon.getStatorCurrent();
    private final StatusSignal<Temperature> pivotTemp = pivotFalcon.getDeviceTemp();

    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);

    // clockwise to intake, counter-clockwise to spit out
    private final SparkMax funnelNeo = new SparkMax(FUNNEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    RelativeEncoder funnelEncoder = funnelNeo.getEncoder();

    public CoralIntakeIOReal() {
        TalonFXConfiguration spinTalonFXConfigs = new TalonFXConfiguration();
        TalonFXConfiguration pivotTalonFXConfigs = new TalonFXConfiguration();
        SparkMaxConfig funnelNEOConfig = new SparkMaxConfig();

        spinTalonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        spinTalonFXConfigs.CurrentLimits.withStatorCurrentLimit(120);
        spinTalonFXConfigs.CurrentLimits.withSupplyCurrentLimit(70);
        spinTalonFXConfigs.CurrentLimits.withSupplyCurrentLowerLimit(40);
        spinTalonFXConfigs.CurrentLimits.withSupplyCurrentLowerTime(0.5);
        spinTalonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        spinTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        spinTalonFXConfigs.Feedback.SensorToMechanismRatio = SPIN_GEAR_RATIO;

        spinFalcon.getConfigurator().apply(spinTalonFXConfigs);

        pivotTalonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        pivotTalonFXConfigs.CurrentLimits.withStatorCurrentLimit(120);
        pivotTalonFXConfigs.CurrentLimits.withSupplyCurrentLimit(70);
        pivotTalonFXConfigs.CurrentLimits.withSupplyCurrentLowerLimit(40);
        pivotTalonFXConfigs.CurrentLimits.withSupplyCurrentLowerTime(0.5);
        pivotTalonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotTalonFXConfigs.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;

        Slot0Configs pivotGains = pivotTalonFXConfigs.Slot0;
        pivotGains.kS = PIVOT_KS;
        pivotGains.kV = PIVOT_KV;
        pivotGains.kG = PIVOT_KG;
        pivotGains.kP = PIVOT_KP;
        pivotGains.kI = PIVOT_KI;
        pivotGains.kD = PIVOT_KD;
        pivotGains.GravityType = GravityTypeValue.Arm_Cosine;

        MotionMagicConfigs motionMagicConfigs = pivotTalonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = PIVOT_CRUISE_VELOCITY_RPS;
        motionMagicConfigs.MotionMagicAcceleration = PIVOT_ACCELERATION_RPS2;

        pivotFalcon.getConfigurator().apply(pivotTalonFXConfigs);
        pivotFalcon.setPosition(PIVOT_MAX_ANGLE.getRotations());

        funnelNEOConfig.secondaryCurrentLimit(80);
        funnelNEOConfig.smartCurrentLimit(40);
        funnelNEOConfig.inverted(false);
        funnelNEOConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        funnelNEOConfig.encoder.positionConversionFactor(1.0 / FUNNEL_GEAR_RATIO);

        funnelNeo.configure(
                funnelNEOConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                spinVelocity,
                spinPosition,
                spinAppliedVoltage,
                spinSupplyCurrent,
                spinStatorCurrent,
                spinTemp,
                pivotVelocity,
                pivotPosition,
                pivotAppliedVoltage,
                pivotSupplyCurrent,
                pivotStatorCurrent,
                pivotTemp);
        spinFalcon.optimizeBusUtilization();
        pivotFalcon.optimizeBusUtilization();
    }

    @Override
    public void setPivotSetpoint(Rotation2d rotations) {
        pivotFalcon.setControl(pivotPositionRequest.withPosition(rotations.getRotations()));
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotFalcon.setControl(pivotVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setRollerVoltage(double voltage) {
        spinFalcon.setControl(spinVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setFunnelVoltage(double voltage) {
        funnelNeo.setVoltage(voltage);
    }

    @Override
    public void updateInputs(CoralIntakeInputs inputs) {
        inputs.spinConnected = BaseStatusSignal.refreshAll(
                        spinVelocity, spinPosition, spinAppliedVoltage, spinSupplyCurrent, spinStatorCurrent, spinTemp)
                .isOK();
        inputs.pivotConnected = BaseStatusSignal.refreshAll(
                        pivotVelocity,
                        pivotPosition,
                        pivotAppliedVoltage,
                        pivotSupplyCurrent,
                        pivotStatorCurrent,
                        pivotTemp)
                .isOK();

        inputs.spinMotorVelocityRPS = spinVelocity.getValueAsDouble();
        inputs.spinMotorAppliedVolts = spinAppliedVoltage.getValueAsDouble();
        inputs.spinSupplyCurrentAmps = spinSupplyCurrent.getValueAsDouble();
        inputs.spinStatorCurrentAmps = spinStatorCurrent.getValueAsDouble();

        inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValueAsDouble());
        inputs.pivotMotorVelocityRPS = pivotVelocity.getValueAsDouble();
        inputs.pivotMotorAppliedVolts = pivotAppliedVoltage.getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.getValueAsDouble();
        inputs.pivotStatorCurrentAmps = pivotStatorCurrent.getValueAsDouble();

        inputs.funnelMotorAppliedVolts = (funnelNeo.getAppliedOutput() * funnelNeo.getBusVoltage());
        inputs.funnelMotorVelocityRPS = funnelEncoder.getVelocity();
        inputs.funnelSupplyCurrentAmps = funnelNeo.getOutputCurrent();
        inputs.funnelStatorCurrentAmps = funnelNeo.getOutputCurrent();
    }

    public void setPivotPID(double kP, double kI, double kD) {
        Slot0Configs configs = new Slot0Configs();
        pivotFalcon.getConfigurator().refresh(configs);
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        pivotFalcon.getConfigurator().apply(configs);
    }

    public void setPivotFF(double kS, double kV, double kG) {
        Slot0Configs configs = new Slot0Configs();
        pivotFalcon.getConfigurator().refresh(configs);
        configs.kS = kS;
        configs.kV = kV;
        configs.kA = kG;
        pivotFalcon.getConfigurator().apply(configs);
    }
}

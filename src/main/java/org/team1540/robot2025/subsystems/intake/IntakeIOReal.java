package org.team1540.robot2025.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class IntakeIOReal implements IntakeIO {

    // rotation of horizontal beams for the intake, clockwise to intake, counter-clockwise to spit out
    private final TalonFX spinFalcon = new TalonFX(IntakeConstants.SPIN_FALCON_ID);

    private final StatusSignal<AngularVelocity> spinVelocity = spinFalcon.getVelocity();
    private final StatusSignal<Angle> spinPosition = spinFalcon.getPosition();
    private final StatusSignal<Voltage> spinAppliedVoltage = spinFalcon.getMotorVoltage();
    private final StatusSignal<Current> spinSupplyCurrent = spinFalcon.getSupplyCurrent();
    private final StatusSignal<Temperature> spinTemp = spinFalcon.getDeviceTemp();
    private final StatusSignal<Current> spinStatorCurrent = spinFalcon.getStatorCurrent();

    // controls position of intake
    private final TalonFX pivotFalcon = new TalonFX(IntakeConstants.PIVOT_FALCON_ID);

    private final StatusSignal<AngularVelocity> pivotVelocity = pivotFalcon.getVelocity();
    private final StatusSignal<Angle> pivotPosition = pivotFalcon.getPosition();
    private final StatusSignal<Voltage> pivotAppliedVoltage = pivotFalcon.getMotorVoltage();
    private final StatusSignal<Current> pivotSupplyCurrent = pivotFalcon.getSupplyCurrent();
    private final StatusSignal<Temperature> pivotTemp = pivotFalcon.getDeviceTemp();
    private final StatusSignal<Current> pivotStatorCurrent = pivotFalcon.getStatorCurrent();

    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0).withSlot(0);

    // clockwise to intake, counter-clockwise to spit out
    private final SparkMax funnelNeo = new SparkMax(IntakeConstants.NEO_ID, SparkLowLevel.MotorType.kBrushless);

    RelativeEncoder funnelEncoder = funnelNeo.getEncoder();

    public IntakeIOReal() {

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

        spinFalcon.getConfigurator().apply(spinTalonFXConfigs);

        pivotTalonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        pivotTalonFXConfigs.CurrentLimits.withStatorCurrentLimit(120);
        pivotTalonFXConfigs.CurrentLimits.withSupplyCurrentLimit(70);
        pivotTalonFXConfigs.CurrentLimits.withSupplyCurrentLowerLimit(40);
        pivotTalonFXConfigs.CurrentLimits.withSupplyCurrentLowerTime(0.5);
        pivotTalonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs pivotConfigs = pivotTalonFXConfigs.Slot0;

        pivotConfigs.kS = IntakeConstants.PIVOT_INTAKE_KS;
        pivotConfigs.kV = IntakeConstants.PIVOT_INTAKE_KV;
        pivotConfigs.kA = IntakeConstants.PIVOT_INTAKE_KA;
        pivotConfigs.kP = IntakeConstants.PIVOT_INTAKE_KP;
        pivotConfigs.kI = IntakeConstants.PIVOT_INTAKE_KI;
        pivotConfigs.kD = IntakeConstants.PIVOT_INTAKE_KD;

        MotionMagicConfigs motionMagicConfigs = pivotTalonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.PIVOT_INTAKE_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.PIVOT_INTAKE_ACCELERATION;

        pivotFalcon.getConfigurator().apply(pivotTalonFXConfigs);

        pivotFalcon.setPosition(0);

        funnelNEOConfig.secondaryCurrentLimit(40);
        funnelNEOConfig.smartCurrentLimit(40);
        funnelNEOConfig.inverted(false);
        funnelNEOConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        funnelNeo.configure(
                funnelNEOConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void setPivot(Rotation2d rotations) {
        pivotFalcon.setControl(pivotPositionRequest.withPosition(rotations.getRotations()));
    }

    @Override
    public void setRollerVoltage(double voltage) {
        spinFalcon.setVoltage(voltage);
    }

    @Override
    public void setFunnelVoltage(double voltage) {
        funnelNeo.setVoltage(voltage);
    }

    @Override
    public void updateInputs(CoralIntakeInputs inputs) {
        BaseStatusSignal.refreshAll(
                spinVelocity,
                spinPosition,
                spinAppliedVoltage,
                spinSupplyCurrent,
                spinTemp,
                spinStatorCurrent,
                pivotVelocity,
                pivotPosition,
                pivotAppliedVoltage,
                pivotSupplyCurrent,
                pivotTemp,
                pivotStatorCurrent);

        inputs.spinMotorPosition = spinPosition.getValueAsDouble();
        inputs.spinMotorVelocityRPS = spinVelocity.getValueAsDouble();
        inputs.spinMotorAppliedVolts = spinAppliedVoltage.getValueAsDouble();
        inputs.spinCurrentAmps = spinStatorCurrent.getValueAsDouble();

        inputs.pivotMotorPosition = pivotPosition.getValueAsDouble();
        inputs.pivotMotorVelocityRPS = pivotVelocity.getValueAsDouble();
        inputs.pivotMotorAppliedVolts = pivotAppliedVoltage.getValueAsDouble();
        inputs.pivotCurrentAmps = pivotStatorCurrent.getValueAsDouble();

        inputs.funnelMotorPosition = funnelEncoder.getPosition();
        inputs.funnelMotorVelocityRPS = funnelEncoder.getVelocity();
        inputs.funnelCurrentAmps = funnelNeo.getOutputCurrent();
        inputs.funnelOutputVoltage = (funnelNeo.getAppliedOutput() * funnelNeo.getBusVoltage());
    }
}

package org.team1540.robot2025.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOReal implements IntakeIO {

    // rotation of horizontal beams for the intake, clockwise to intake, counter-clockwise to spit out
    private final TalonFX spinFalcon = new TalonFX(IntakeConstants.SPIN_FALCON_ID);

    // controls position of intake
    private final TalonFX pivotFalcon = new TalonFX(IntakeConstants.PIVOT_FALCON_ID);

    private final MotionMagicVoltage motorRequest = new MotionMagicVoltage(0).withSlot(0);

    // clockwise to intake, counter-clockwise to spit out
    private final SparkMax funnelNeo = new SparkMax(IntakeConstants.NEO_ID, SparkLowLevel.MotorType.kBrushless);

    private StatusCode spinFalconStatus;

    private StatusCode pivotFalconStatus;

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

        Slot0Configs spinConfigs = spinTalonFXConfigs.Slot0;

        spinFalcon.getConfigurator().apply(spinConfigs);

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

        pivotFalcon.getConfigurator().apply(pivotConfigs);

        pivotFalcon.setPosition(0);

        funnelNEOConfig.secondaryCurrentLimit(70);
        funnelNEOConfig.inverted(false);
        funnelNEOConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        funnelNeo.configure(funnelNEOConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void setPivot(Rotation2d rotations) {
        pivotFalcon.setControl(motorRequest.withPosition(rotations.getRotations()));
    }

    @Override
    public void setRollerSpeed(double speed) {
        spinFalcon.set(speed);
    }

    @Override
    public void setFunnelSpeed(double speed) {
        funnelNeo.set(speed);
    }

    @Override
    public void updateInputs(CoralIntakeInputs inputs) {

        inputs.spinMotorPosition = spinFalcon.getPosition().getValueAsDouble();
        inputs.spinMotorVelocityRPS = spinFalcon.getVelocity().getValueAsDouble();
        inputs.spinMotorAppliedVolts = spinFalcon.getMotorVoltage().getValueAsDouble();
        inputs.spinCurrentAmps = spinFalcon.getStatorCurrent().getValueAsDouble();
        spinFalconStatus = spinFalcon.getMotorOutputStatus().getStatus();

        inputs.pivotMotorPosition = pivotFalcon.getPosition().getValueAsDouble();
        inputs.pivotMotorVelocityRPS = pivotFalcon.getVelocity().getValueAsDouble();
        inputs.pivotMotorAppliedVolts = pivotFalcon.getMotorVoltage().getValueAsDouble();
        inputs.pivotCurrentAmps = pivotFalcon.getStatorCurrent().getValueAsDouble();
        pivotFalconStatus = pivotFalcon.getMotorOutputStatus().getStatus();

        inputs.funnelMotorPosition = funnelNeo.getEncoder().getPosition();
        inputs.funnelMotorVelocityRPS = funnelNeo.getEncoder().getVelocity();
        inputs.funnelCurrentAmps = funnelNeo.getOutputCurrent();
        inputs.funnelOutputVoltage = (funnelNeo.getAppliedOutput() * funnelNeo.getBusVoltage());

    }
}

package org.team1540.robot2025.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOReal implements IntakeIO {

    // rotation of horizontal beams for the intake, clockwise to intake, counter-clockwise to spit out
    private final TalonFX spinFalcon = new TalonFX(IntakeConstants.SPIN_FALCON_ID);

    // controls position of intake
    private final TalonFX pivotFalcon = new TalonFX(IntakeConstants.PIVOT_FALCON_ID);

    private final MotionMagicVoltage motorRequest = new MotionMagicVoltage(0).withSlot(0);

    // clockwise to intake, counter-clockwise to spit out
    private final SparkMax neo = new SparkMax(IntakeConstants.NEO_ID, SparkLowLevel.MotorType.kBrushless);

    public IntakeIOReal() {

        TalonFXConfiguration spinTalonFXConfigs = new TalonFXConfiguration();
        TalonFXConfiguration pivotTalonFXConfigs = new TalonFXConfiguration();

        spinTalonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        spinTalonFXConfigs.CurrentLimits.withStatorCurrentLimit(IntakeConstants.SPIN_FALCON_CURRENT_LIMIT);
        spinTalonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        spinTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotTalonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        pivotTalonFXConfigs.CurrentLimits.withStatorCurrentLimit(IntakeConstants.PIVOT_FALCON_CURRENT_LIMIT);
        spinTalonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs spinConfigs = pivotTalonFXConfigs.Slot0;
        Slot1Configs pivotConfigs = pivotTalonFXConfigs.Slot1;

        pivotConfigs.kS = IntakeConstants.INTAKE_KS;
        pivotConfigs.kV = IntakeConstants.INTAKE_KV;
        pivotConfigs.kA = IntakeConstants.INTAKE_KA;
        pivotConfigs.kP = IntakeConstants.INTAKE_KP;
        pivotConfigs.kI = IntakeConstants.INTAKE_KI;
        pivotConfigs.kD = IntakeConstants.INTAKE_KD;

        MotionMagicConfigs motionMagicConfigs = pivotTalonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.INTAKE_ACCELERATION;

        spinFalcon.getConfigurator().apply(spinConfigs);
        pivotFalcon.getConfigurator().apply(pivotConfigs);

        pivotFalcon.setPosition(0);
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
    public void setNEOSpeed(double speed) {
        neo.set(speed);
    }

    @Override
    public void updateInputs(CoralIntakeInputs inputs) {

        inputs.spinMotorPosition = spinFalcon.getPosition().getValueAsDouble();
        inputs.spinMotorVelocityRPS = spinFalcon.getVelocity().getValueAsDouble();
        inputs.spinMotorAppliedVolts = spinFalcon.getMotorVoltage().getValueAsDouble();

        inputs.pivotMotorPosition = pivotFalcon.getPosition().getValueAsDouble();
        inputs.pivotMotorVelocityRPS = pivotFalcon.getVelocity().getValueAsDouble();
        inputs.pivotMotorAppliedVolts = pivotFalcon.getMotorVoltage().getValueAsDouble();

        inputs.neoMotorAppliedOutput = neo.getAppliedOutput();
    }
}

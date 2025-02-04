package org.team1540.robot2025.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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

        var talonFXConfigs = new TalonFXConfiguration();
        var slot0configs = talonFXConfigs.Slot0;

        slot0configs.kS = IntakeConstants.GRABBER_KS;
        slot0configs.kV = IntakeConstants.GRABBER_KV;
        slot0configs.kA = IntakeConstants.GRABBER_KA;
        slot0configs.kP = IntakeConstants.GRABBER_KP;
        slot0configs.kI = IntakeConstants.GRABBER_KI;
        slot0configs.kD = IntakeConstants.GRABBER_KD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.GRABBER_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.GRABBER_ACCELERATION;

        pivotFalcon.getConfigurator().apply(slot0configs);
        spinFalcon.getConfigurator().apply(slot0configs);

//        positionFalcon.

        //        topFalcon.setInverted(false);
        //        bottomFalcon.setInverted(true);
        //        neo.setInverted(false);

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

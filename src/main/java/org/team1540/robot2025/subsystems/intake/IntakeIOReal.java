package org.team1540.robot2025.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import org.team1540.robot2025.Constants;

public class IntakeIOReal implements IntakeIO {

    // rotation of horizontal beams for the intake, clockwise to intake, counter-clockwise to spit out
    private final TalonFX topFalcon = new TalonFX(Constants.IntakeConstants.TOP_FALCON_ID);

    // controls height of intake
    private final TalonFX bottomFalcon = new TalonFX(Constants.IntakeConstants.BOTTOM_FALCON_ID);

    private final PositionVoltage motorRequest = new PositionVoltage(0).withSlot(0);

    // clockwise to intake, counter-clockwise to spit out
    private final SparkMax neo = new SparkMax(Constants.IntakeConstants.NEO_ID, SparkLowLevel.MotorType.kBrushless);

    public IntakeIOReal() {

        var talonFXConfigs = new TalonFXConfiguration();
        var slot0configs = talonFXConfigs.Slot0;

        slot0configs.kS = Constants.IntakeConstants.GRABBER_KS;
        slot0configs.kV = Constants.IntakeConstants.GRABBER_KV;
        slot0configs.kA = Constants.IntakeConstants.GRABBER_KA;
        slot0configs.kP = Constants.IntakeConstants.GRABBER_KP;
        slot0configs.kI = Constants.IntakeConstants.GRABBER_KI;
        slot0configs.kD = Constants.IntakeConstants.GRABBER_KD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.IntakeConstants.GRABBER_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.IntakeConstants.GRABBER_ACCELERATION;

        bottomFalcon.getConfigurator().apply(slot0configs);

        //        topFalcon.setInverted(false);
        //        bottomFalcon.setInverted(true);
        //        neo.setInverted(false);

        bottomFalcon.setPosition(0);
    }

    @Override
    public void setPosition(Rotation2d rotations) {
        bottomFalcon.setControl(motorRequest.withPosition(rotations.getRotations()));
    }

    @Override
    public void setSpeed(double speed) {
        topFalcon.set(speed);
        neo.set(speed);
        // told by Ryan that they will always be in sync, will separate if needed
        // also need to figure out if they are going to be different speeds, might be easier when separated
    }

    @Override
    public void updateInputs(CoralIntakeInputs inputs) {

        inputs.topMotorPosition = topFalcon.getPosition().getValueAsDouble();
        inputs.topMotorVelocityRPS = topFalcon.getVelocity().getValueAsDouble();
        inputs.topMotorAppliedVolts = topFalcon.getMotorVoltage().getValueAsDouble();

        inputs.bottomMotorPosition = bottomFalcon.getPosition().getValueAsDouble();
        inputs.bottomMotorVelocityRPS = bottomFalcon.getVelocity().getValueAsDouble();
        inputs.bottomMotorAppliedVolts = bottomFalcon.getMotorVoltage().getValueAsDouble();

        inputs.neoMotorAppliedOutput = neo.getAppliedOutput();
    }
}

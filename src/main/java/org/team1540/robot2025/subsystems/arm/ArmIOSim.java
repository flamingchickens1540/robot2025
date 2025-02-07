package org.team1540.robot2025.subsystems.arm;

import static org.team1540.robot2025.Constants.LOOP_PERIOD_SECS;
import static org.team1540.robot2025.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {

    // fields
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            GEAR_RATIO,
            ARM_MOMENT_OF_INERTIA_KGM2,
            ARM_LENGTH_METERS,
            MIN_ANGLE.getRadians(),
            MAX_ANGLE.getRadians(),
            true,
            ArmState.STOW.angle.get().getRadians());

    private double armAppliedVolts = 0.0;
    private final ProfiledPIDController controller =
            new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(MAX_VELOCITY_RPS, MAX_ACCELERATION));
    private ArmFeedforward feedforward = new ArmFeedforward(KS, KG, KV);
    private boolean isClosedLoop;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (isClosedLoop) {
            armAppliedVolts = controller.calculate(Units.radiansToRotations(armSim.getAngleRads()))
                    + feedforward.calculate(
                            Units.rotationsToRadians(controller.getSetpoint().position),
                            controller.getSetpoint().velocity);
        }

        armSim.setInputVoltage(armAppliedVolts);
        armSim.update(LOOP_PERIOD_SECS);

        inputs.position = Rotation2d.fromRadians(armSim.getAngleRads());
        inputs.appliedVolts = armAppliedVolts;
        inputs.supplyCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec());
    }

    @Override
    public void setSetpoint(Rotation2d setpoint) {
        if (!isClosedLoop)
            controller.reset(
                    Units.radiansToRotations(armSim.getAngleRads()),
                    Units.radiansToRotations(armSim.getVelocityRadPerSec()));
        isClosedLoop = true;
        controller.setGoal(new TrapezoidProfile.State(setpoint.getRotations(), 0));
    }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        armAppliedVolts = volts;
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    @Override
    public void configFF(double kS, double kV, double kG) {
        feedforward = new ArmFeedforward(kS, kG, kV);
    }
}

package org.team1540.robot2025.subsystems.climber;

import static org.team1540.robot2025.Constants.LOOP_PERIOD_SECS;
import static org.team1540.robot2025.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.team1540.robot2025.subsystems.arm.Arm;

public class ClimberIOSim implements ClimberIO {
    private static final double SIM_KP = 300;
    private static final double SIM_KI = 50;
    private static final double SIM_KD = 1;
    private static final double SIM_KS = 0.03;
    private static final double SIM_KG = 0.37;
    private static final double SIM_KV = 0.8;

    // fields
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            GEAR_RATIO,
            ARM_MOMENT_OF_INERTIA_KGM2,
            ARM_LENGTH_METERS,
            MIN_ANGLE.getRadians(),
            MAX_ANGLE.getRadians(),
            true,
            Arm.ArmState.STOW.position().getRadians());

    private double armAppliedVolts = 0.0;
    private final ProfiledPIDController controller = new ProfiledPIDController(
            SIM_KP, SIM_KI, SIM_KD, new TrapezoidProfile.Constraints(CRUISE_VELOCITY_RPS, MAX_ACCEL_RPS2));
    private ArmFeedforward feedforward = new ArmFeedforward(SIM_KS, SIM_KG, SIM_KV);
    private boolean isClosedLoop;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (isClosedLoop) {
            armAppliedVolts = controller.calculate(Units.radiansToRotations(armSim.getAngleRads()))
                    + feedforward.calculate(
                            Units.rotationsToRadians(controller.getSetpoint().position),
                            Units.rotationsToRadians(controller.getSetpoint().velocity));
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
}

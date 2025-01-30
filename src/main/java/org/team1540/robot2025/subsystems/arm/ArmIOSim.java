package org.team1540.robot2025.subsystems.arm;

import static org.team1540.robot2025.Constants.Arm.*;
import static org.team1540.robot2025.Constants.LOOP_PERIODIC_SECS;

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
            ARM_MOMENT_OF_INERTIA,
            ARM_LENGTH_METERS,
            MIN_ANGLE.getRadians(),
            MAX_ANGLE.getRadians(),
            true,
            STARTING_ANGLE.getRadians());

    private double armAppliedVolts = 0.0;
    private final ProfiledPIDController controller =
            new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    private final ArmFeedforward feedforward = new ArmFeedforward(KS, KG, KV);
    private boolean isClosedLoop;
    private TrapezoidProfile.State goalState;

    // methods
    //TODO: constructor
    public void updateInputs(ArmIOInputs inputs) {
        if (isClosedLoop) {
            armAppliedVolts = controller.calculate(armSim.getAngleRads(), inputs.rotation2d.getRadians())
                    + feedforward.calculate(
                            Units.rotationsToDegrees(controller.getSetpoint().position),
                            controller.getSetpoint().velocity);
        }

        armSim.setInputVoltage(armAppliedVolts);
        armSim.update(LOOP_PERIODIC_SECS);

        // TODO: should I rename rotation2d to call it position rotation 2d
        inputs.rotation2d = Rotation2d.fromRadians(armSim.getAngleRads());
        inputs.appliedVolts = armAppliedVolts;
        inputs.currentAmps = armSim.getCurrentDrawAmps();
        inputs.velocityRPM = armSim.getVelocityRadPerSec() * 60 / (2 * Math.PI); // converting to rpm;
    }

    public void setPosition(Rotation2d position) {
        controller.reset(
                Units.radiansToRotations(armSim.getAngleRads()),
                Units.radiansToRotations(armSim.getVelocityRadPerSec()));
        isClosedLoop = true;
        goalState = new TrapezoidProfile.State(position.getRotations(), 0);
    }

    public void configPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    public void setVoltage(double volts) {
        isClosedLoop = false;
        armAppliedVolts = volts;
    }
}

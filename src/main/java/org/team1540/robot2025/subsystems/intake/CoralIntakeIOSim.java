package org.team1540.robot2025.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.team1540.robot2025.Constants;

import static org.team1540.robot2025.subsystems.intake.CoralIntakeConstants.*;

public class CoralIntakeIOSim implements CoralIntakeIO {
    private final DCMotorSim spinSim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            DCMotor.getFalcon500Foc(1), 0.001, SPIN_GEAR_RATIO),
                    DCMotor.getFalcon500Foc(1));
    private final DCMotorSim funnelSim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            DCMotor.getNeo550(1), 0.001, FUNNEL_GEAR_RATIO),
                    DCMotor.getNeo550(1));
    private final SingleJointedArmSim pivotSim =
            new SingleJointedArmSim(
                    DCMotor.getKrakenX60(1),
                    PIVOT_GEAR_RATIO,
                    0.02,
                    Units.inchesToMeters(4),
                    PIVOT_MIN_ANGLE.getRadians(),
                    PIVOT_MAX_ANGLE.getRadians(),
                    true,
                    PIVOT_MAX_ANGLE.getRadians());

    private double spinAppliedVolts = 0.0;
    private double funnelAppliedVolts = 0.0;
    private double pivotAppliedVolts = 0.0;

    private final ProfiledPIDController pivotController =
            new ProfiledPIDController(PIVOT_KP, PIVOT_KI, PIVOT_KD, new TrapezoidProfile.Constraints(PIVOT_CRUISE_VELOCITY_RPS, PIVOT_ACCELERATION_RPS2));
    private ArmFeedforward pivotFeedforward = new ArmFeedforward(PIVOT_KS, PIVOT_KG, PIVOT_KV);
    private boolean isPivotClosedLoop;

    public CoralIntakeIOSim() {
        pivotController.reset(PIVOT_MAX_ANGLE.getRotations());
    }

    @Override
    public void updateInputs(CoralIntakeInputs inputs) {
        if (isPivotClosedLoop) {
            pivotAppliedVolts = pivotController.calculate(Units.radiansToRotations(pivotSim.getAngleRads()))
                    + pivotFeedforward.calculate(
                            Units.rotationsToRadians(pivotController.getSetpoint().position),
                            Units.rotationsToRadians(pivotController.getSetpoint().velocity));
        }
        spinSim.setInputVoltage(spinAppliedVolts);
        funnelSim.setInputVoltage(funnelAppliedVolts);
        pivotSim.setInputVoltage(pivotAppliedVolts);
        spinSim.update(Constants.LOOP_PERIOD_SECS);
        funnelSim.update(Constants.LOOP_PERIOD_SECS);
        pivotSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.spinMotorAppliedVolts = spinAppliedVolts;
        inputs.spinMotorVelocityRPS = spinSim.getAngularVelocityRPM() / 60.0;
        inputs.spinStatorCurrentAmps = spinSim.getCurrentDrawAmps();
        inputs.spinSupplyCurrentAmps = spinSim.getCurrentDrawAmps();

        inputs.funnelMotorAppliedVolts = funnelAppliedVolts;
        inputs.funnelMotorVelocityRPS = funnelSim.getAngularVelocityRPM() / 60.0;
        inputs.funnelStatorCurrentAmps = funnelSim.getCurrentDrawAmps();
        inputs.funnelSupplyCurrentAmps = funnelSim.getCurrentDrawAmps();

        inputs.pivotMotorAppliedVolts = pivotAppliedVolts;
        inputs.pivotPosition = Rotation2d.fromRadians(pivotSim.getAngleRads());
        inputs.pivotMotorVelocityRPS = Units.rotationsToRadians(pivotSim.getVelocityRadPerSec()) / 60.0;
        inputs.pivotStatorCurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.pivotSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();
    }

    @Override
    public void setRollerVoltage(double voltage) {
        spinAppliedVolts = voltage;
    }

    @Override
    public void setFunnelVoltage(double voltage) {
        funnelAppliedVolts = voltage;
    }

    @Override
    public void setPivotVoltage(double voltage) {
        isPivotClosedLoop = false;
        pivotAppliedVolts = voltage;
    }

    @Override
    public void setPivotSetpoint(Rotation2d rotations) {
        if (!isPivotClosedLoop) {
            pivotController.reset(Units.radiansToRotations(pivotSim.getAngleRads()), Units.radiansToRotations(pivotSim.getVelocityRadPerSec()));
        }
        isPivotClosedLoop = true;
        pivotController.setGoal(new TrapezoidProfile.State(rotations.getRotations(), 0));
    }

    @Override
    public void setPivotPID(double kP, double kI, double kD) {
        pivotController.setPID(kP, kI, kD);
    }

    @Override
    public void setPivotFF(double kS, double kV, double kG) {
        pivotFeedforward = new ArmFeedforward(kS, kG, kV);
    }
}

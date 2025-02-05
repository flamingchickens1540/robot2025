package org.team1540.robot2025.subsystems.elevator;

import static org.team1540.robot2025.Constants.LOOP_PERIOD_SECS;
import static org.team1540.robot2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            GEAR_RATIO,
            SIM_CARRIAGE_MASS_KG,
            SPROCKET_RADIUS_M,
            MIN_HEIGHT,
            MAX_HEIGHT,
            true,
            MIN_HEIGHT);
    private double appliedVolts = 0.0;
    private final ProfiledPIDController controller = new ProfiledPIDController(
            KP, KI, KD, new TrapezoidProfile.Constraints(CRUISE_VELOCITY_MPS, MAXIMUM_ACCELERATION_MPS2));
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(KS, KG, KV);
    private boolean isClosedLoop;
    private TrapezoidProfile.State setpoint;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (isClosedLoop) {
            appliedVolts = controller.calculate(elevatorSim.getPositionMeters(), setpoint)
                    + feedforward.calculate(controller.getSetpoint().velocity);
        }
        elevatorSim.setInputVoltage(appliedVolts);
        elevatorSim.update(LOOP_PERIOD_SECS);

        inputs.positionMeters = new double[] {elevatorSim.getPositionMeters()};
        inputs.velocityMPS = new double[] {elevatorSim.getVelocityMetersPerSecond()};
        inputs.appliedVolts = new double[] {appliedVolts};
        inputs.supplyCurrentAmps = new double[] {elevatorSim.getCurrentDrawAmps()};
        inputs.atUpperLimit = elevatorSim.hasHitUpperLimit();
        inputs.atLowerLimit = elevatorSim.hasHitLowerLimit();
    }

    @Override
    public void setSetpoint(double setpointMeters) {
        isClosedLoop = true;
        setpoint = new TrapezoidProfile.State(setpointMeters, 0.0);
    }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        appliedVolts = volts;
    }
}

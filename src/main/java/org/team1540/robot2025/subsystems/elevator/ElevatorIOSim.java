package org.team1540.robot2025.subsystems.elevator;

import static org.team1540.robot2025.subsystems.elevator.ElevatorConstants.*;
import static org.team1540.robot2025.Constants.LOOP_PERIOD_SECS;

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
    private double elevatorAppliedVolts = 0.0;
    private final ProfiledPIDController controller = new ProfiledPIDController(
            KP, KI, KD, new TrapezoidProfile.Constraints(CRUISE_VELOCITY_MPS, MAXIMUM_ACCELERATION_MPS2));
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(KS, KG, KV);
    private boolean isClosedLoop;
    private TrapezoidProfile.State setpoint;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (isClosedLoop) {
            elevatorAppliedVolts = controller.calculate(elevatorSim.getPositionMeters(), setpoint)
                    + feedforward.calculate(controller.getSetpoint().velocity);
        }
        elevatorSim.setInputVoltage(elevatorAppliedVolts);
        elevatorSim.update(LOOP_PERIOD_SECS);

        inputs.positionMeters = elevatorSim.getPositionMeters();
        inputs.velocityMPS = elevatorSim.getVelocityMetersPerSecond();
        inputs.leaderAppliedVolts = elevatorAppliedVolts;
        inputs.leaderCurrentAmps = elevatorSim.getCurrentDrawAmps();
        inputs.atUpperLimit = elevatorSim.hasHitUpperLimit();
        inputs.atLowerLimit = elevatorSim.hasHitLowerLimit();
    }

    @Override
    public void setSetpointMeters(double position) {
        isClosedLoop = true;
        setpoint = new TrapezoidProfile.State(position, 0.0);
    }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        elevatorAppliedVolts = volts;
    }
}

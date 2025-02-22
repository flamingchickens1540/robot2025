package org.team1540.robot2025.subsystems.elevator;

import static org.team1540.robot2025.Constants.LOOP_PERIOD_SECS;
import static org.team1540.robot2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private static final double SIM_KP = 300;
    private static final double SIM_KI = 0;
    private static final double SIM_KD = 0;
    private static final double SIM_KS = 0.03178;
    private static final double SIM_KV = 2.562;
    private static final double SIM_KG = 0;

    private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            GEAR_RATIO,
            SIM_CARRIAGE_MASS_KG,
            SPROCKET_RADIUS_M,
            MIN_HEIGHT_M,
            MAX_HEIGHT_M,
            true,
            MIN_HEIGHT_M);
    private double appliedVolts = 0.0;
    private final ProfiledPIDController controller = new ProfiledPIDController(
            SIM_KP, SIM_KI, SIM_KD, new TrapezoidProfile.Constraints(CRUISE_VELOCITY_MPS, MAXIMUM_ACCELERATION_MPS2));
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(SIM_KS, SIM_KG, SIM_KV);
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

        inputs.connection = new boolean[] {true, true};
        inputs.positionMeters = new double[] {elevatorSim.getPositionMeters()};
        inputs.velocityMPS = new double[] {elevatorSim.getVelocityMetersPerSecond()};
        inputs.appliedVolts = new double[] {appliedVolts};
        inputs.supplyCurrentAmps = new double[] {elevatorSim.getCurrentDrawAmps()};
        inputs.statorCurrentAmps = new double[] {elevatorSim.getCurrentDrawAmps()};
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

package org.team1540.robot2025.subsystems.elevator;

import static org.team1540.robot2025.Constants.Elevator.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;

public class Elevator implements Subsystem {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double setpointMeters;

    private static boolean hasInstance = false;

    private Elevator(ElevatorIO elevatorIO) {
        if (hasInstance) throw new IllegalStateException("Instance of elevator already exists");
        hasInstance = true;
        this.io = elevatorIO;
    }

    public static Elevator createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real elevator on simulated robot", false);
        }
        return new Elevator(new ElevatorIOTalonFX());
    }

    public static Elevator createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated elevator on real robot", false);
        }
        return new Elevator(new ElevatorIOSim());
    }

    public static Elevator createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy elevator on real robot", false);
        }
        return new Elevator(new ElevatorIO() {});
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (RobotState.isDisabled()) stop();
    }

    public void setElevatorPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, MIN_HEIGHT, MAX_HEIGHT);
        setpointMeters = positionMeters;
        io.setSetpointMeters(setpointMeters);
    }

    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpointMeters, inputs.positionMeters, 0.1);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    @AutoLogOutput(key = "Elevator/setpoint")
    public double getSetpoint() {
        return setpointMeters;
    }

    public double getPosition() {
        return inputs.positionMeters;
    }

    public double getVelocity() {
        return inputs.velocityMPS;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    public boolean getUpperLimit() {
        return inputs.atUpperLimit;
    }

    public boolean getLowerLimit() {
        return inputs.atLowerLimit;
    }

    public void holdPosition() {
        setElevatorPosition(inputs.positionMeters);
    }

    public Command setpointCommand(ElevatorState state) {
        return Commands.runOnce(() -> setElevatorPosition(state.elevatorHeight), this).until(this::isAtSetpoint);
    }
}

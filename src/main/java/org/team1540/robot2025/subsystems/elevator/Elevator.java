package org.team1540.robot2025.subsystems.elevator;

import static org.team1540.robot2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.util.MechanismVisualiser;

import java.util.function.DoubleSupplier;


public class Elevator implements Subsystem {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LinearFilter positionFilter = LinearFilter.movingAverage(10);
    private double setpointMeters;
    private final double DEADZONE = 0.2;

    private static boolean hasInstance = false;

    private Elevator(ElevatorIO elevatorIO) {
        if (hasInstance) throw new IllegalStateException("Instance of elevator already exists");
        hasInstance = true;
        this.io = elevatorIO;
    }



    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (RobotState.isDisabled()) stop();

        MechanismVisualiser.setElevatorPosition(inputs.positionMeters[0]);
        positionFilter.calculate(inputs.positionMeters[0]);
    }

    public void setPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, MIN_HEIGHT, MAX_HEIGHT);
        setpointMeters = positionMeters;
        io.setSetpoint(setpointMeters);
    }

    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpointMeters, positionFilter.lastValue(), POS_ERR_TOLERANCE_METERS) || (inputs.atLowerLimit && setpointMeters <= 0);
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
        return inputs.positionMeters[0];
    }

    public double getVelocity() {
        return inputs.velocityMPS[0];
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
        setPosition(inputs.positionMeters[0]);
    }

    public Command setpointCommand(ElevatorState state) {
        return Commands.runOnce(() -> setPosition(state.elevatorHeight), this)
                .until(this::isAtSetpoint);
    }

    public Command manualCommand(DoubleSupplier input) {
        return Commands.runEnd(() -> {
            double val = input.getAsDouble();
            if (Math.abs(val) > DEADZONE) {
                if (val < 0) {
                    val += DEADZONE;
                } else {
                    val -= DEADZONE;
                }
                setVoltage(val * 15);
            } else {
                holdPosition();
            }
        },
                this::holdPosition,
                this
        );
    }

    public Command runSetpointCommand(ElevatorState state) {
        return Commands.run(() -> setPosition(state.elevatorHeight), this);
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
}

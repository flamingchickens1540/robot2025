package org.team1540.robot2025.subsystems.elevator;

import static org.team1540.robot2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.commands.CharacterizationCommands;
import org.team1540.robot2025.services.MechanismVisualizer;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
    private static boolean hasInstance = false;

    public enum ElevatorState {
        STOW(new LoggedTunableNumber("Elevator/Setpoints/Base", MIN_HEIGHT_M)),
        FUNNEL(new LoggedTunableNumber("Elevator/Setpoints/Funnel", 0.242)),
        L1_BACK(new LoggedTunableNumber("Elevator/Setpoints/L1Back", 0.7)),
        L2(new LoggedTunableNumber("Elevator/Setpoints/L2", 0.55)),
        L3(new LoggedTunableNumber("Elevator/Setpoints/L3", 0.92)),
        L4(new LoggedTunableNumber("Elevator/Setpoints/L4", MAX_HEIGHT_M)),
        BARGE(new LoggedTunableNumber("Elevator/Setpoints/Barge", MAX_HEIGHT_M)),
        GROUND_ALGAE(new LoggedTunableNumber("Elevator/Setpoints/GroundAlgae", 0.33)),
        REEF_ALGAE_LOW(new LoggedTunableNumber("Elevator/Setpoints/ReefAlgaeLow", 0.6)),
        REEF_ALGAE_HIGH(new LoggedTunableNumber("Elevator/Setpoints/ReefAlgaeHigh", 0.98)),
        PROCESSOR(new LoggedTunableNumber("Elevator/Setpoints/Processor", 0)), // TODO: get value
        STOW_ALGAE(new LoggedTunableNumber("Elevator/Setpoints/StowAlgae", 0.03));

        public final DoubleSupplier height;

        ElevatorState(DoubleSupplier height) {
            this.height = height;
        }
    }

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double setpointMeters;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", KV);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", KG);

    private final Alert leaderDisconnectedAlert = new Alert("Elevator leader disconnected", Alert.AlertType.kError);
    private final Alert followerDisconnectedAlert = new Alert("Elevator follower disconnected", Alert.AlertType.kError);

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

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);

        MechanismVisualizer.getInstance().setElevatorPosition(inputs.positionMeters[0]);

        leaderDisconnectedAlert.set(!inputs.connection[0]);
        followerDisconnectedAlert.set(!inputs.connection[1]);
    }

    public void setPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, MIN_HEIGHT_M, MAX_HEIGHT_M);
        setpointMeters = positionMeters;
        io.setSetpoint(setpointMeters);
    }

    @AutoLogOutput(key = "Elevator/AtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpointMeters, inputs.positionMeters[0], POS_ERR_TOLERANCE_M)
                || (inputs.atLowerLimit && setpointMeters <= 0);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    @AutoLogOutput(key = "Elevator/Setpoint")
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

    public void resetPosition(double positionMeters) {
        io.resetPosition(positionMeters);
    }

    public Command commandToSetpoint(ElevatorState state) {
        return (Commands.run(() -> setPosition(state.height.getAsDouble()), this)
                        .until(this::isAtSetpoint))
                .handleInterrupt(this::holdPosition);
    }

    public Command manualCommand(DoubleSupplier input) {
        return Commands.runEnd(() -> setVoltage(input.getAsDouble()), this::holdPosition, this);
    }

    public Command runSetpointCommand(ElevatorState state) {
        return Commands.run(() -> setPosition(state.height.getAsDouble()), this);
    }

    public Command zeroCommand() {
        return manualCommand(() -> -0.5)
                .until(() -> inputs.statorCurrentAmps[0] > 20)
                .andThen(Commands.runOnce(() -> resetPosition(0)));
    }

    public Command feedforwardCharacterizationCommand() {
        return CharacterizationCommands.feedforward(this::setVoltage, this::getVelocity, this);
    }

    public static Elevator createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real elevator on simulated robot", false);
        }
        return new Elevator(new ElevatorIOTalonFX());
    }

    public static Elevator createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated elevator on real robot", false);
        }
        return new Elevator(new ElevatorIOSim());
    }

    public static Elevator createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy elevator on real robot", false);
        }
        return new Elevator(new ElevatorIO() {});
    }
}

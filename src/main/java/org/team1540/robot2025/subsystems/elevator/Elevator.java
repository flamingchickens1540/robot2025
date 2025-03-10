package org.team1540.robot2025.subsystems.elevator;

import static org.team1540.robot2025.subsystems.arm.ArmConstants.CRUISE_VELOCITY_RPS;
import static org.team1540.robot2025.subsystems.arm.ArmConstants.MAX_ACCEL_RPS2;
import static org.team1540.robot2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
import org.team1540.robot2025.util.LoggedTracer;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
    private static boolean hasInstance = false;

    public enum ElevatorState {
        STOW(new LoggedTunableNumber("Elevator/Setpoints/Base", MIN_HEIGHT_M)),
        FUNNEL(new LoggedTunableNumber("Elevator/Setpoints/Funnel", 0.242)),
        GROUND_CORAL(new LoggedTunableNumber("Elevator/Setpoints/GroundCoral", 0)),
        L1_BACK(new LoggedTunableNumber("Elevator/Setpoints/L1Back", 0.2)),
        L1_FRONT(new LoggedTunableNumber("Elevator/Setpoints/L1Front", 0.2)),
        L2_BACK(new LoggedTunableNumber("Elevator/Setpoints/L2Back", 0.55)),
        L2_FRONT(new LoggedTunableNumber("Elevator/Setpoints/L2Front", 0.56)),
        L3_BACK(new LoggedTunableNumber("Elevator/Setpoints/L3Back", 1.0)),
        L3_FRONT(new LoggedTunableNumber("Elevator/Setpoints/L3Front", 0.95)),
        L4_BACK(new LoggedTunableNumber("Elevator/Setpoints/L4Back", MAX_HEIGHT_M)),
        L4_FRONT(new LoggedTunableNumber("Elevator/Setpoints/L4Front", MAX_HEIGHT_M - Units.inchesToMeters(2.25))),
        BARGE(new LoggedTunableNumber("Elevator/Setpoints/Barge", MAX_HEIGHT_M)),
        GROUND_ALGAE(new LoggedTunableNumber("Elevator/Setpoints/GroundAlgae", 0.42)),
        REEF_ALGAE_LOW_BACK(new LoggedTunableNumber("Elevator/Setpoints/ReefAlgaeLowBack", 0.8)),
        REEF_ALGAE_LOW_FRONT(new LoggedTunableNumber("Elevator/Setpoints/ReefAlgaeLowFront", 0.8)),
        REEF_ALGAE_HIGH_BACK(new LoggedTunableNumber("Elevator/Setpoints/ReefAlgaeHighBack", 1.2)),
        REEF_ALGAE_HIGH_FRONT(new LoggedTunableNumber("Elevator/Setpoints/ReefAlgaeHighFront", 1.2)),
        PROCESSOR(new LoggedTunableNumber("Elevator/Setpoints/Processor", 0.254)), // TODO: get value
        STOW_ALGAE(new LoggedTunableNumber("Elevator/Setpoints/StowAlgae", 0.03));

        public final DoubleSupplier height;

        ElevatorState(DoubleSupplier height) {
            this.height = height;
        }
    }

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", KD);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", KV);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", KG);

    private final Alert leaderDisconnectedAlert = new Alert("Elevator leader disconnected", Alert.AlertType.kError);
    private final Alert followerDisconnectedAlert = new Alert("Elevator follower disconnected", Alert.AlertType.kError);

    private final TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(CRUISE_VELOCITY_RPS, MAX_ACCEL_RPS2));

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double setpointMeters;

    private final Debouncer atBottomDebounce = new Debouncer(0.25);
    private boolean atBottom = false;

    private Elevator(ElevatorIO elevatorIO) {
        if (hasInstance) throw new IllegalStateException("Instance of elevator already exists");
        hasInstance = true;
        this.io = elevatorIO;
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        atBottom = atBottomDebounce.calculate(
                inputs.statorCurrentAmps[0] >= 80 && Math.abs(inputs.velocityMPS[0]) <= 0.05);

        if (RobotState.isDisabled()) stop();

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);

        MechanismVisualizer.getInstance().setElevatorPosition(inputs.positionMeters[0]);

        leaderDisconnectedAlert.set(!inputs.connection[0]);
        followerDisconnectedAlert.set(!inputs.connection[1]);

        LoggedTracer.record("Elevator");
    }

    public void setPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, 0.0, MAX_HEIGHT_M);
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

    public double timeToSetpoint(double setpoint) {
        trapezoidProfile.calculate(
                0.0,
                new TrapezoidProfile.State(getPosition(), inputs.velocityMPS[0]),
                new TrapezoidProfile.State(setpoint, 0));
        return trapezoidProfile.totalTime();
    }

    @AutoLogOutput(key = "Elevator/TimeToSetpoint")
    public double timeToSetpoint() {
        return timeToSetpoint(getSetpoint());
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
                .until(this::isAtSetpoint));
        //                .handleInterrupt(this::holdPosition);
    }

    public Command manualCommand(DoubleSupplier input) {
        return Commands.runEnd(() -> setVoltage(input.getAsDouble() * 12.0), this::holdPosition, this);
    }

    public Command runSetpointCommand(ElevatorState state) {
        return Commands.run(() -> setPosition(state.height.getAsDouble()), this);
    }

    public Command zeroCommand() {
        return Commands.deadline(
                        Commands.waitUntil(() -> atBottom).andThen(Commands.waitSeconds(0.25)),
                        commandToSetpoint(ElevatorState.STOW)
                                .onlyIf(() -> getPosition() > ElevatorState.STOW.height.getAsDouble())
                                .withTimeout(0.5)
                                .andThen(Commands.runOnce(() -> setVoltage(-2.5), this)))
                .andThen(Commands.runOnce(() -> resetPosition(0)), commandToSetpoint(ElevatorState.STOW));
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

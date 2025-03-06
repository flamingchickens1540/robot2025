package org.team1540.robot2025.subsystems.climber;

import static org.team1540.robot2025.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.services.MechanismVisualizer;
import org.team1540.robot2025.util.LoggedTracer;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {
    private static boolean hasInstance = false;

    public enum ClimberState {
        CLIMB(new LoggedTunableNumber("Climber/Setpoints/ClimbDegrees", 70)),
        ;

        private final DoubleSupplier positionDegrees;

        ClimberState(DoubleSupplier positionDegrees) {
            this.positionDegrees = positionDegrees;
        }

        public Rotation2d position() {
            return Rotation2d.fromDegrees(positionDegrees.getAsDouble());
        }
    }

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private Rotation2d setpoint = new Rotation2d();

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Climber/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/kD", KD);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Climber/kG", KG);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Climber/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Climber/kV", KV);

    private Climber(ClimberIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of climber already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {
        LoggedTracer.reset();

        // update + process inputs!
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        MechanismVisualizer.getInstance().setClimberRotation(inputs.position);

        if (RobotState.isDisabled()) {
            io.setVoltage(0);
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);

        LoggedTracer.record("Climber");
    }

    public void holdPosition() {
        setPosition(inputs.position);
    }

    public void setPosition(Rotation2d position) {
        setpoint = Rotation2d.fromRotations(
                MathUtil.clamp(position.getRotations(), MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations()));
        io.setSetpoint(setpoint);
    }

    public void resetPosition(Rotation2d position) {
        io.resetPivotPosition(position);
    }

    public Rotation2d getPosition() {
        return inputs.position;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    @AutoLogOutput(key = "Climber/AtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpoint.getRotations(), inputs.position.getRotations(), ERROR_TOLERANCE.getRotations());
    }

    @AutoLogOutput(key = "Climber/Setpoint")
    public Rotation2d getSetpoint() {
        return setpoint;
    }

    public Command commandToSetpoint(ClimberState state) {
        return (Commands.run(() -> setPosition(state.position()), this).until(this::isAtSetpoint))
                .handleInterrupt(this::holdPosition);
    }

    public Command manualCommand(DoubleSupplier input) {
        return Commands.runEnd(() -> io.setVoltage(input.getAsDouble() * 12.0), () -> io.setVoltage(0), this);
    }

    public Command climbCommand(DoubleSupplier input) {
        return Commands.runEnd(
                        () -> io.setVoltage(Math.max(input.getAsDouble() * 12.0, 0)), () -> io.setVoltage(0), this)
                .until(() -> getPosition().getDegrees()
                        > ClimberState.CLIMB.position().getDegrees());
    }

    public static Climber createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real climber on simulated robot", false);
        }
        return new Climber(new ClimberIOTalonFX());
    }

    public static Climber createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated climber on real robot", false);
        }
        return new Climber(new ClimberIOSim());
    }

    public static Climber createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy climber on real robot", false);
        }
        return new Climber(new ClimberIO() {});
    }
}

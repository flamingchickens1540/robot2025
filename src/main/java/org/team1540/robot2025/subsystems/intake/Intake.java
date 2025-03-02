package org.team1540.robot2025.subsystems.intake;

import static org.team1540.robot2025.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.services.MechanismVisualizer;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
    private static boolean hasInstance = false;

    public enum IntakeState {
        STOW(new LoggedTunableNumber("Intake/Setpoints/Stow/AngleDegrees", PIVOT_MAX_ANGLE.getDegrees())),
        INTAKE(new LoggedTunableNumber("Intake/Setpoints/Intake/AngleDegrees", PIVOT_MIN_ANGLE.getDegrees())),
        EJECT(new LoggedTunableNumber("Intake/Setpoints/Eject/AngleDegrees", 60)),
        L1(new LoggedTunableNumber("Intake/Setpoints/L1/AngleDegrees", 30));

        private final DoubleSupplier pivotPosition;

        IntakeState(DoubleSupplier pivotPositionDeg) {
            this.pivotPosition = pivotPositionDeg;
        }

        public Rotation2d pivotPosition() {
            return Rotation2d.fromDegrees(pivotPosition.getAsDouble());
        }
    }

    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private final LoggedTunableNumber pivotKP = new LoggedTunableNumber("Intake/kP", PIVOT_KP);
    private final LoggedTunableNumber pivotKI = new LoggedTunableNumber("Intake/kI", PIVOT_KI);
    private final LoggedTunableNumber pivotKD = new LoggedTunableNumber("Intake/kD", PIVOT_KD);
    private final LoggedTunableNumber pivotKS = new LoggedTunableNumber("Intake/kS", PIVOT_KS);
    private final LoggedTunableNumber pivotKV = new LoggedTunableNumber("Intake/kV", PIVOT_KV);
    private final LoggedTunableNumber pivotKG = new LoggedTunableNumber("Intake/kG", PIVOT_KG);

    private final Alert pivotDisconnectedAlert = new Alert("Intake pivot disconnected", Alert.AlertType.kError);
    private final Alert rollerDisconnectedAlert = new Alert("Intake roller disconnected", Alert.AlertType.kError);
    private final Alert funnelDisconnectedAlert = new Alert("Intake funnel disconnected", Alert.AlertType.kError);

    private Rotation2d pivotSetpoint = PIVOT_MIN_ANGLE;

    private final TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(PIVOT_CRUISE_VELOCITY_RPS, PIVOT_ACCELERATION_RPS2));

    private Intake(IntakeIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of intake already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        MechanismVisualizer.getInstance().setIntakeRotation(inputs.pivotPosition);

        if (DriverStation.isDisabled()) stopAll();

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setPivotPID(pivotKP.get(), pivotKI.get(), pivotKD.get()),
                pivotKP,
                pivotKI,
                pivotKD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setPivotFF(pivotKS.get(), pivotKV.get(), pivotKG.get()),
                pivotKS,
                pivotKV,
                pivotKG);

        pivotDisconnectedAlert.set(!inputs.pivotConnected);
        rollerDisconnectedAlert.set(!inputs.spinConnected);
        funnelDisconnectedAlert.set(!inputs.funnelConnected);
    }

    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    public void setFunnelVoltage(double voltage) {
        io.setFunnelVoltage(voltage);
    }

    public void setPivotPosition(Rotation2d rotations) {
        pivotSetpoint = rotations;
        io.setPivotSetpoint(rotations);
    }

    public Rotation2d getPivotPosition() {
        return inputs.pivotPosition;
    }

    public void resetPivotPosition(Rotation2d rotations) {
        io.resetPivotPosition(rotations);
    }

    public void setPivotVoltage(double voltage) {
        io.setPivotVoltage(voltage);
    }

    public void stopAll() {
        setRollerVoltage(0);
        setFunnelVoltage(0);
        setPivotVoltage(0);
    }

    public void holdPivot() {
        setPivotPosition(inputs.pivotPosition);
    }

    @AutoLogOutput(key = "Intake/PivotAtSetpoint")
    public boolean isPivotAtSetpoint() {
        return MathUtil.isNear(pivotSetpoint.getDegrees(), inputs.pivotPosition.getDegrees(), 3.0);
    }

    @AutoLogOutput(key = "Intake/PivotSetpoint")
    public Rotation2d getPivotSetpoint() {
        return pivotSetpoint;
    }

    public boolean hasCoral() {
        return inputs.sensorTripped;
    }

    @AutoLogOutput(key = "Intake/TimeToSetpoint")
    public double timeToSetpoint() {
        return timeToSetpoint(pivotSetpoint);
    }

    public double timeToSetpoint(Rotation2d setpoint) {
        trapezoidProfile.calculate(
                0.0,
                new TrapezoidProfile.State(getPivotPosition().getRotations(), inputs.pivotMotorVelocityRPS),
                new TrapezoidProfile.State(setpoint.getRotations(), 0));
        return trapezoidProfile.totalTime();
    }

    public Command commandToSetpoint(IntakeState state) {
        return (Commands.run(() -> setPivotPosition(state.pivotPosition()), this)
                        .until(this::isPivotAtSetpoint))
                .handleInterrupt(this::holdPivot);
    }

    public Command commandRunRoller(double percent) {
        return Commands.startEnd(() -> this.setRollerVoltage(percent * 12), () -> this.setRollerVoltage(0), this);
    }

    public Command commandRunFunnel(double percent) {
        return Commands.startEnd(() -> this.setFunnelVoltage(percent * 12), () -> this.setFunnelVoltage(0), this);
    }

    public Command commandRunRollerFunnel(double rollerPercent, double funnelPercent) {
        return Commands.startEnd(
                () -> {
                    this.setRollerVoltage(rollerPercent * 12);
                    this.setFunnelVoltage(funnelPercent * 12);
                },
                () -> {
                    this.setRollerVoltage(0);
                    this.setFunnelVoltage(0);
                });
    }

    public Command zeroCommand() {
        return Commands.runOnce(() -> setPivotVoltage(0.3 * 12))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(Commands.waitUntil(() -> Math.abs(inputs.pivotStatorCurrentAmps) > 20)
                        .andThen(Commands.runOnce(() -> resetPivotPosition(Rotation2d.fromDegrees(90))))
                        .andThen(commandToSetpoint(IntakeState.STOW)));
    }

    public static Intake createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real coral intake on simulated robot", false);
        }
        return new Intake(new IntakeIOReal());
    }

    public static Intake createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated coral intake on real robot", false);
        }
        return new Intake(new IntakeIOSim());
    }

    public static Intake createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy coral intake on real robot", false);
        }
        return new Intake(new IntakeIO() {});
    }
}

package org.team1540.robot2025.subsystems.intake;

import static org.team1540.robot2025.subsystems.intake.CoralIntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class CoralIntake extends SubsystemBase {
    private static boolean hasInstance = false;

    public enum CoralIntakeState {
        STOW(
                new LoggedTunableNumber("CoralIntake/Setpoints/Stow/AngleDegrees", PIVOT_MAX_ANGLE.getDegrees()),
                new LoggedTunableNumber("CoralIntake/Setpoints/Stow/RollerVoltage", 0.0),
                new LoggedTunableNumber("CoralIntake/Setpoints/Stow/FunnelVoltage", 0.0)),
        INTAKE(
                new LoggedTunableNumber("CoralIntake/Setpoints/Intake/AngleDegrees", PIVOT_MIN_ANGLE.getDegrees()),
                new LoggedTunableNumber("CoralIntake/Setpoints/Intake/RollerVoltage", 12.0),
                new LoggedTunableNumber("CoralIntake/Setpoints/Intake/FunnelVoltage", 12.0)),
        EJECT(
                new LoggedTunableNumber("CoralIntake/Setpoints/Eject/AngleDegrees", PIVOT_MIN_ANGLE.getDegrees()),
                new LoggedTunableNumber("CoralIntake/Setpoints/Eject/RollerVoltage", -12.0),
                new LoggedTunableNumber("CoralIntake/Setpoints/Eject/FunnelVoltage", -12.0));

        private final DoubleSupplier pivotPosition;
        public final DoubleSupplier rollerVoltage;
        public final DoubleSupplier funnelVoltage;

        CoralIntakeState(DoubleSupplier pivotPositionDeg, DoubleSupplier rollerVoltage, DoubleSupplier funnelVoltage) {
            this.pivotPosition = pivotPositionDeg;
            this.rollerVoltage = rollerVoltage;
            this.funnelVoltage = funnelVoltage;
        }

        public Rotation2d pivotPosition() {
            return Rotation2d.fromDegrees(pivotPosition.getAsDouble());
        }
    }

    private final CoralIntakeIO io;
    private final CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();

    private final LoggedTunableNumber pivotKP = new LoggedTunableNumber("CoralIntake/kP", PIVOT_KP);
    private final LoggedTunableNumber pivotKI = new LoggedTunableNumber("CoralIntake/kI", PIVOT_KI);
    private final LoggedTunableNumber pivotKD = new LoggedTunableNumber("CoralIntake/kD", PIVOT_KD);
    private final LoggedTunableNumber pivotKS = new LoggedTunableNumber("CoralIntake/kS", PIVOT_KS);
    private final LoggedTunableNumber pivotKV = new LoggedTunableNumber("CoralIntake/kV", PIVOT_KV);
    private final LoggedTunableNumber pivotKG = new LoggedTunableNumber("CoralIntake/kG", PIVOT_KG);

    private final Alert pivotDisconnectedAlert = new Alert("Intake pivot disconnected", Alert.AlertType.kError);
    private final Alert rollerDisconnectedAlert = new Alert("Intake roller disconnected", Alert.AlertType.kError);
    private final Alert funnelDisconnectedAlert = new Alert("Intake funnel disconnected", Alert.AlertType.kError);

    @AutoLogOutput(key = "CoralIntake/PivotSetpoint")
    private Rotation2d pivotSetpoint = PIVOT_MIN_ANGLE;

    private CoralIntake(CoralIntakeIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of intake already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralIntake", inputs);

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

    @AutoLogOutput(key = "CoralIntake/PivotAtSetpoint")
    public boolean isPivotAtSetpoint() {
        return MathUtil.isNear(pivotSetpoint.getDegrees(), inputs.pivotPosition.getDegrees(), 3.0);
    }

    public Command commandToSetpoint(CoralIntakeState state) {
        return Commands.run(
                        () -> {
                            setPivotPosition(state.pivotPosition());
                            setRollerVoltage(state.rollerVoltage.getAsDouble());
                            setFunnelVoltage(state.funnelVoltage.getAsDouble());
                        },
                        this)
                .until(this::isPivotAtSetpoint);
    }

    public static CoralIntake createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real coral intake on simulated robot", false);
        }
        return new CoralIntake(new CoralIntakeIOReal());
    }

    public static CoralIntake createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated coral intake on real robot", false);
        }
        return new CoralIntake(new CoralIntakeIOSim());
    }

    public static CoralIntake createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy coral intake on real robot", false);
        }
        return new CoralIntake(new CoralIntakeIO() {});
    }
}

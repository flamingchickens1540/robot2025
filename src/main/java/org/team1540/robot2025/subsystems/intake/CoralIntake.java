package org.team1540.robot2025.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;

public class CoralIntake extends SubsystemBase {
    private static boolean hasInstance = false;

    private final IntakeIO io;
    private final CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();

    private final Alert pivotDisconnectedAlert = new Alert("Intake pivot disconnected", Alert.AlertType.kError);
    private final Alert rollerDisconnectedAlert = new Alert("Intake roller disconnected", Alert.AlertType.kError);
    private final Alert funnelDisconnectedAlert = new Alert("Intake funnel disconnected", Alert.AlertType.kError);

    public CoralIntake(IntakeIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of intake already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Intake", inputs);

        pivotDisconnectedAlert.set(!inputs.pivotConnected);
        rollerDisconnectedAlert.set(!inputs.spinConnected);
        funnelDisconnectedAlert.set(!inputs.funnelConnected);
    }

    public void setSpinVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    public void setFunnelVoltage(double voltage) {
        io.setFunnelVoltage(voltage);
    }

    public void setPivotPosition(Rotation2d rotations) {
        io.setPivotSetpoint(rotations);
    }

    public void setPivotVoltage(double voltage) {
        io.setPivotVoltage(voltage);
    }

    public void stopAll() {
        setSpinVoltage(0);
        setFunnelVoltage(0);
        setPivotVoltage(0);
    }

    public Command commandSetPivot(Rotation2d rotations) {
        return Commands.runOnce(() -> setPivotPosition(rotations));
    }

    public Command commandSetSpinSpeed(double speed) {
        return Commands.run(() -> setSpinVoltage(speed));
    }

    public Command commandSetFunnelSpeed(double speed) {
        return Commands.run(() -> setFunnelVoltage(speed));
    }

    public static CoralIntake createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real indexer on simulated robot", false);
        }
        return new CoralIntake(new IntakeIOReal());
    }

    public static CoralIntake createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy indexer on real robot", false);
        }
        return new CoralIntake(new IntakeIO() {});
    }
}

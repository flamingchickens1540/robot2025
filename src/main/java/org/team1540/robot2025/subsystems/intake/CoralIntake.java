package org.team1540.robot2025.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;

public class CoralIntake extends SubsystemBase {

    private final IntakeIO io;
    private final CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();

    public CoralIntake(IntakeIO io) {
        this.io = io;
    }

    public static CoralIntake createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real indexer on simulated robot", false);
        }
        return new CoralIntake(new IntakeIOReal());
    }

    public static CoralIntake createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy indexer on real robot", false);
        }
        return new CoralIntake(new IntakeIO() {});
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Intake", inputs);
    }

    public void setSpinSpeed(double speed) {
        io.setRollerVoltage(speed);
    }

    public void setFunnelSpeed(double speed) {
        io.setFunnelVoltage(speed);
    }

    public void setPivot(Rotation2d rotations) {
        io.setPivot(rotations);
    }

    public Command commandSetPivot(Rotation2d rotations) {
        return Commands.runOnce(() -> setPivot(rotations));
    }

    public Command commandSetSpinSpeed(double speed) {
        return Commands.run(() -> setSpinSpeed(speed));
    }

    public Command commandSetFunnelSpeed(double speed) {
        return Commands.run(() -> setFunnelSpeed(speed));
    }
}

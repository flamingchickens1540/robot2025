package org.team1540.robot2025.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public static Intake createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real indexer on simulated robot", false);
        }
        return new Intake(new IntakeIOReal());
    }

    public static Intake createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy indexer on real robot", false);
        }
        return new Intake(new IntakeIO() {});
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Intake", inputs);
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setPosition(Rotation2d rotations) {
        io.setPosition(rotations);
    }
}

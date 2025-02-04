package org.team1540.robot2025.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2025.subsystems.intake.Intake;

public class IntakePositionCommand extends Command {

    private final Intake intake;
    private final Rotation2d rotations;

    public IntakePositionCommand(Intake intake, Rotation2d rotations) {
        this.intake = intake;
        this.rotations = rotations;
    }

    @Override
    public void initialize() {
        intake.setPosition(rotations);
    }
}

package org.team1540.robot2025.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2025.subsystems.intake.CoralIntake;

public class IntakePositionCommand extends Command {

    private final CoralIntake coralIntake;
    private final Rotation2d rotations;

    public IntakePositionCommand(CoralIntake coralIntake, Rotation2d rotations) {
        this.coralIntake = coralIntake;
        this.rotations = rotations;
    }

    @Override
    public void initialize() {
        coralIntake.setPivot(rotations);
    }
}

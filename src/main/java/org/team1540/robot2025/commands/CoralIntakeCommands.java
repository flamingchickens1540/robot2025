package org.team1540.robot2025.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2025.subsystems.intake.CoralIntake;

public class CoralIntakeCommands extends Command {

    private final CoralIntake coralIntake;

    public CoralIntakeCommands(CoralIntake coralIntake) {
        this.coralIntake = coralIntake;
    }

    public Command commandSetPivot(Rotation2d rotations) {
        return Commands.runOnce(()->coralIntake.setPivot(rotations));
    }

    public Command commandSetSpinSpeed(double speed) {
        return Commands.run(()->coralIntake.setSpinSpeed(speed));
    }

    public Command commandSetFunnelSpeed(double speed) {
        return Commands.run(()->coralIntake.setFunnelSpeed(speed));
    }
}

package org.team1540.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2025.subsystems.intake.CoralIntake;

public class IntakeSpeedCommand extends Command {

    private final CoralIntake coralIntake;
    private final int speed;

    public IntakeSpeedCommand(CoralIntake coralIntake, int speed) {
        this.coralIntake = coralIntake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        coralIntake.setSpinSpeed(speed);
    }
}

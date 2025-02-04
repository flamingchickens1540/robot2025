package org.team1540.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2025.subsystems.intake.Intake;

public class IntakeSpeedCommand extends Command {

    private final Intake intake;
    private final int speed;

    public IntakeSpeedCommand(Intake intake, int speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.setSpinSpeed(speed);
    }
}

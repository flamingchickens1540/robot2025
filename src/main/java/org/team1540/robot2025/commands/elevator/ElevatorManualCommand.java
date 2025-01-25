package org.team1540.robot2025.commands.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2025.subsystems.elevator.Elevator;

public class ElevatorManualCommand extends Command {
    private final XboxController copilot;
    private final Elevator elevator;
    private final double deadzone = 0.2;

    public ElevatorManualCommand(Elevator elevator,  XboxController copilot) {
        this.elevator = elevator;
        this.copilot = copilot;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double percent = copilot.getRightY();
        if (Math.abs(percent) > deadzone) {
            elevator.setVoltage(copilot.getRightY() * 12);
        } else {
            elevator.holdPosition();
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}

package org.team1540.robot2025;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.robot2025.subsystems.leds.Leds;

public class RobotContainer {
    Leds leds = new Leds();
    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        configureButtonBindings();
        configureAutoRoutines();
    }

    private void configureButtonBindings() {
        new Trigger(RobotState::isEnabled).whileTrue(leds.showRSLState());
    }

    private void configureAutoRoutines() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}

package org.team1540.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        configureButtonBindings();
        configureAutoRoutines();
    }

    private void configureButtonBindings() {}

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

package org.team1540.robot2025.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumble {

    private static Rumble instance = null;

    public static Rumble getInstance() {
        if (instance == null) instance = new Rumble();
        return instance;
    }

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    public static Command startStopTimed(Runnable start, Runnable end, double duration, Subsystem... requirements) {
        return Commands.race(Commands.waitSeconds(duration), Commands.startEnd(start, end, requirements));
    }

    public static Command rumbleCommandTimed(XboxController controller, double amount, double duration) {
        return startStopTimed(
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, amount),
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0),
                duration);
    }

    public static Command rumbleCommandTimed(CommandXboxController controller, double amount, double duration) {
        return rumbleCommandTimed(controller.getHID(), amount, duration);
    }

    public static Command rumbleCommand(XboxController controller, double amount) {
        return Commands.startEnd(
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, amount),
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0));
    }

    public static Command rumbleCommand(CommandXboxController controller, double amount) {
        return rumbleCommand(controller.getHID(), amount);
    }

    public Command rumbleDriver() {
        return rumbleCommand(driver, 1);
    }

    public Command rumbleDriverTimed(double duration) {
        return rumbleCommandTimed(driver, 1, duration);
    }

    public Command setDriverRumble(GenericHID.RumbleType type, double value) {
        return Commands.runOnce(() -> driver.setRumble(type, value));
    }

    public Command rumbleCopilot() {
        return rumbleCommand(copilot, 1);
    }

    public Command rumbleCopilotTimed(double duration) {
        return rumbleCommandTimed(copilot, 1, duration);
    }

    public Command setCopilotRumble(GenericHID.RumbleType type, double value) {
        return Commands.runOnce(() -> copilot.setRumble(type, value));
    }
}

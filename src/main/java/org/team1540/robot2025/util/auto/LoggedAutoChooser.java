package org.team1540.robot2025.util.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class LoggedAutoChooser {
    private final LoggedDashboardChooser<String> dashboardChooser;

    private final AutoChooser autoChooser = new AutoChooser();

    public LoggedAutoChooser(String key) {
        dashboardChooser = new LoggedDashboardChooser<>(key);
        dashboardChooser.addDefaultOption("Nothing", "Nothing");
    }

    public void update() {
        autoChooser.select(dashboardChooser.get());
    }

    public void addRoutine(String name, Supplier<AutoRoutine> routine) {
        dashboardChooser.addOption(name, name);
        autoChooser.addRoutine(name, routine);
    }

    public void addCmd(String name, Supplier<Command> command) {
        dashboardChooser.addOption(name, name);
        autoChooser.addCmd(name, command);
    }

    public Command selectedCommand() {
        return autoChooser.selectedCommand();
    }

    public Command selectedCommandScheduler() {
        return autoChooser.selectedCommandScheduler();
    }
}

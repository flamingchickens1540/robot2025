package org.team1540.robot2025.util.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.lang.reflect.Field;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class LoggedAutoChooser extends LoggedNetworkInput {
    private final String key;
    private final AutoChooser chooser = new AutoChooser();
    private final Map<String, Supplier<Command>> options;

    private String selectedValue = null;
    private final Field selectedValueField;

    private final LoggableInputs inputs = new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
            table.put(key, selectedValue);
        }

        @Override
        public void fromLog(LogTable table) {
            selectedValue = table.get(key, selectedValue);
        }
    };

    @SuppressWarnings("unchecked")
    public LoggedAutoChooser(String key) {
        this.key = key;
        SmartDashboard.putData(key, chooser);

        try {
            Field mapField = AutoChooser.class.getDeclaredField("autoRoutines");
            mapField.setAccessible(true);
            options = (Map<String, Supplier<Command>>) mapField.get(chooser);
        } catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        try {
            selectedValueField = AutoChooser.class.getDeclaredField("selected");
            selectedValueField.setAccessible(true);
        } catch (NoSuchFieldException | SecurityException e) {
            throw new RuntimeException(e);
        }

        periodic();
        Logger.registerDashboardInput(this);
    }

    public void addRoutine(String name, Supplier<AutoRoutine> generator) {
        options.put(name, () -> generator.get().cmd());
        chooser.addRoutine(name, generator);
    }

    public void addCmd(String name, Supplier<Command> cmd) {
        options.put(name, cmd);
        chooser.addCmd(name, cmd);
    }

    public Command selectedCommandScheduler() {
        return Commands.defer(() -> options.get(selectedValue).get().asProxy(), Set.of());
    }

    public Command selectedCommand() {
        return options.get(selectedValue).get().withName(selectedValue);
    }

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            try {
                selectedValue = (String) selectedValueField.get(chooser);
            } catch (IllegalAccessException | IllegalArgumentException e) {
                e.printStackTrace();
                selectedValue = "Nothing";
            }
        }
        Logger.processInputs(prefix + "/SmartDashboard", inputs);
    }
}

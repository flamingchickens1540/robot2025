package org.team1540.robot2025.subsystems.leds;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Value;
import static org.team1540.robot2025.subsystems.leds.LedConstants.LEDS_LENGTH;
import static org.team1540.robot2025.subsystems.leds.LedConstants.LEDS_PWM_PORT;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Leds extends SubsystemBase {
    private final AddressableLED ledStrip = new AddressableLED(LEDS_PWM_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_LENGTH);

    private final List<LedWindow> children = new ArrayList<>();
    public final LedWindow viewFull = new LedWindow(0, LEDS_LENGTH - 1);
    public final LedWindow viewTop = new LedWindow(LEDS_LENGTH - 6, LEDS_LENGTH - 1);

    public Leds() {
        ledStrip.setLength(buffer.getLength());
        ledStrip.start();
        viewFull.setDefaultPattern(
                LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Value.one().div(Second.of(5))));
    }

    @Override
    public void periodic() {
        for (LedWindow child : children) {
            child.apply();
        }
        ledStrip.setData(buffer);
    }

    public static Color getAllianceColor() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? Color.kBlue
                : Color.kRed;
    }

    public class LedWindow extends SubsystemBase {
        private final AddressableLEDBufferView view;
        private LEDPattern activePattern;
        private LEDPattern defaultPattern;

        private LedWindow(int start, int end) {
            view = buffer.createView(start, end);
            children.add(this);
            this.setName("LedWindow" + start + "to" + end);
        }

        private void apply() {
            if (activePattern != null) {
                this.activePattern.applyTo(view);
            } else if (defaultPattern != null) {
                this.defaultPattern.applyTo(view);
            }
        }

        public Command commandShowPattern(LEDPattern pattern) {
            return Commands.startEnd(() -> this.activePattern = pattern, () -> this.activePattern = null, this)
                    .ignoringDisable(true);
        }

        public Command commandShowPattern(Supplier<LEDPattern> pattern) {
            return Commands.startEnd(() -> this.activePattern = pattern.get(), () -> this.activePattern = null, this)
                    .ignoringDisable(true);
        }

        public void setDefaultPattern(LEDPattern pattern) {
            this.defaultPattern = pattern;
        }

        public Command showRSLState() {
            LEDPattern pattern = LEDPattern.solid(new Color("#ff3700"));
            return commandShowPattern(pattern.synchronizedBlink(RobotController::getRSLState));
        }
    }
}

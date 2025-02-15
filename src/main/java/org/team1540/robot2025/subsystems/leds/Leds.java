package org.team1540.robot2025.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Objects;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Value;
import static org.team1540.robot2025.subsystems.leds.LedConstants.LEDS_LENGTH;
import static org.team1540.robot2025.subsystems.leds.LedConstants.LEDS_PWM_PORT;

public class Leds extends SubsystemBase {
    private static final Supplier<LEDPattern> fallbackPattern = () ->
            LEDPattern.rainbow(255, 255)
                    .scrollAtRelativeSpeed(Value.one().div(Second.of(5)));

    private final AddressableLED ledStrip = new AddressableLED(LEDS_PWM_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_LENGTH);
    private LEDPattern defaultPattern = fallbackPattern.get();

    public Leds() {
        ledStrip.setLength(buffer.getLength());
        ledStrip.start();
        this.setDefaultCommand(Commands.run(() -> defaultPattern.applyTo(buffer), this).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        ledStrip.setData(buffer);
    }

    public void setDefaultPattern(LEDPattern pattern) {
        this.defaultPattern = Objects.requireNonNullElseGet(pattern, fallbackPattern);
    }

    public Command commandShowPattern(LEDPattern pattern) {
        return Commands.run(() -> pattern.applyTo(this.buffer), this).ignoringDisable(true);
    }

    public Command showRSLState() {
        LEDPattern pattern = LEDPattern.solid(new Color("#ff3700"));
        return commandShowPattern(pattern.synchronizedBlink(RobotController::getRSLState));
    }
}

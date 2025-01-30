package org.team1540.robot2025.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team1540.robot2025.subsystems.leds.LedConstants.*;

public class Leds extends SubsystemBase {
    private final AddressableLED ledStrip = new AddressableLED(LEDS_PWM_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_LENGTH);

    public Leds() {
        ledStrip.setLength(buffer.getLength());
        ledStrip.setData(buffer);
        ledStrip.start();
    }

    @Override
    public void periodic() {
        ledStrip.setData(buffer);
    }

    public void setPatternAll(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    public void setPattern(LEDPattern pattern, int start, int end) {

    }

    public Command showPattern(LEDPattern pattern) {
        return Commands.runOnce(() -> this.setPatternAll(pattern));
    }

    public Command showRSLState() {
        LEDPattern pattern = LEDPattern.solid(new Color("#FF1A00"));
        return showPattern(pattern.synchronizedBlink(RobotController::getRSLState));
    }
}

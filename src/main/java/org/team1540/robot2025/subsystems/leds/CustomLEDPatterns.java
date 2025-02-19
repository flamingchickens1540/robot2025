package org.team1540.robot2025.subsystems.leds;

import static edu.wpi.first.units.Units.Microseconds;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;

public class CustomLEDPatterns {

    public static LEDPattern movingRainbow(Frequency velocity) {
        return movingRainbow(velocity, 255, 255);
    }

    public static LEDPattern movingRainbow(Frequency velocity, int saturation, int value) {
        final double periodMicros = velocity.asPeriod().in(Microseconds);

        return (reader, writer) -> {
            final double step = 180.0 / reader.getLength();
            long now = RobotController.getTime();

            double t = (now / periodMicros);
            for (int i = 0; i < reader.getLength(); i++) {
                int offset = (int) (i * step + t * 180) % 180;
                writer.setHSV(i, offset, saturation, value);
            }
        };
    }
}

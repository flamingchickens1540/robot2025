package org.team1540.robot2025.subsystems.leds;

import static edu.wpi.first.units.Units.Microseconds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.DoubleSupplier;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2025.subsystems.elevator.ElevatorConstants;

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

    public static LEDPattern drivetrainSpeed(Color color) {
        return LEDPattern.solid(color).mask(LEDPattern.progressMaskLayer(() -> {
            ChassisSpeeds robotSpeed = RobotState.getInstance().getRobotVelocity();
            return Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
                    / DrivetrainConstants.MAX_LINEAR_SPEED_MPS;
        }));
    }

    public static LEDPattern elevatorHeight(Color color, DoubleSupplier elevatorHeightMeters) {
        return LEDPattern.solid(color)
                .mask(LEDPattern.progressMaskLayer(
                        () -> elevatorHeightMeters.getAsDouble() / ElevatorConstants.MAX_HEIGHT_M));
    }
}

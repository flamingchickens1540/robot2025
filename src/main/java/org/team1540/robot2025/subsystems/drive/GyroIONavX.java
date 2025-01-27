package org.team1540.robot2025.subsystems.drive;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

import static edu.wpi.first.units.Units.Kilograms;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
    private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI, (int) Drivetrain.ODOMETRY_FREQUENCY);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIONavX() {
        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(navX::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navX.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(-value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}

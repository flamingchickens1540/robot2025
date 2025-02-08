package org.team1540.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.team1540.robot2025.SimState;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
        gyroSimulation.setRotation(Rotation2d.kZero);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
                gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        inputs.odometryYawTimestamps = SimState.getInstance().getSimulationOdometryTimestamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }
}

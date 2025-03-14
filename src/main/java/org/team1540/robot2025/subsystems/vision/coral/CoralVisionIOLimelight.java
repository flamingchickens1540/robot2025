package org.team1540.robot2025.subsystems.vision.coral;

import static org.team1540.robot2025.subsystems.vision.coral.CoralVisionConstants.PIPELINE_INDEX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import org.team1540.robot2025.util.LimelightHelpers;

public class CoralVisionIOLimelight implements CoralVisionIO {
    private final String name;

    public CoralVisionIOLimelight(String name) {
        this.name = name;
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        LimelightHelpers.setLEDMode_PipelineControl(name);
        LimelightHelpers.setPipelineIndex(name, PIPELINE_INDEX);
    }

    @Override
    public void updateInputs(CoralVisionIOInputs inputs) {
        // Connection status based on whether an update has happened in the last 250 ms
        inputs.connected = ((RobotController.getFPGATime() - LimelightHelpers.getLatency_Pipeline(name)) / 1000) < 250;
        inputs.hasDetection = LimelightHelpers.getTV(name);
        double latestDetectionTimestampSecs =
                LimelightHelpers.getLimelightNTTableEntry(name, "tv").getLastChange();
        inputs.latestDetection = new CoralDetection(
                latestDetectionTimestampSecs,
                Rotation2d.fromDegrees(LimelightHelpers.getTX(name)),
                Rotation2d.fromDegrees(LimelightHelpers.getTY(name)),
                LimelightHelpers.getTA(name));
    }

    @Override
    public String getName() {
        return name;
    }
}

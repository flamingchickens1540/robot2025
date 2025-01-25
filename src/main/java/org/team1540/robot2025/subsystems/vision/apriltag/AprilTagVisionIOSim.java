package org.team1540.robot2025.subsystems.vision.apriltag;

import static org.team1540.robot2025.Constants.Vision.*;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.team1540.robot2025.RobotState;

public class AprilTagVisionIOSim extends AprilTagVisionIOPhoton {
    private final VisionSystemSim visionSim;

    public AprilTagVisionIOSim(String cameraName, Transform3d cameraTransform) {
        super(cameraName, cameraTransform);

        this.visionSim = new VisionSystemSim(cameraName);
        visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);

        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(SIM_RES_WIDTH, SIM_RES_HEIGHT, SIM_DIAGONAL_FOV);
        properties.setFPS(SIM_FPS);
        properties.setAvgLatencyMs(SIM_AVG_LATENCY_MS);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, properties);
        visionSim.addCamera(cameraSim, cameraTransform);
    }

    public void updateInputs(AprilTagVisionIOInputs inputs) {
        visionSim.update(RobotState.getInstance().getEstimatedPose());
        super.updateInputs(inputs);
    }
}

package org.team1540.robot2025.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.*;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team1540.robot2025.FieldConstants;

public class AprilTagVisionIOPhoton extends AprilTagVisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d cameraTransformMeters;

    protected final PhotonPoseEstimator poseEstimator;

    private final Set<Integer> lastSeenTagIDs = new HashSet<>();

    public AprilTagVisionIOPhoton(String cameraName, Transform3d cameraTransformMeters) {
        super(cameraName);
        this.camera = new PhotonCamera(cameraName);
        this.cameraTransformMeters = cameraTransformMeters;

        poseEstimator = new PhotonPoseEstimator(
                FieldConstants.aprilTagLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraTransformMeters);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        List<PoseObservation> poseObservations = new ArrayList<>();
        lastSeenTagIDs.clear();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> poseEstimatorResult = poseEstimator.update(result);

            if (poseEstimatorResult.isPresent()) {
                Pose3d robotPose = poseEstimatorResult.get().estimatedPose;
                double totalDistance = 0.0;
                double totalAmbiguity = 0.0;
                for (PhotonTrackedTarget target : poseEstimatorResult.get().targetsUsed) {
                    totalDistance +=
                            target.getBestCameraToTarget().getTranslation().getNorm();
                    totalAmbiguity += target.poseAmbiguity;
                    lastSeenTagIDs.add(target.fiducialId);
                }

                poseObservations.add(new PoseObservation(
                        robotPose,
                        poseEstimatorResult.get().targetsUsed.size(),
                        totalDistance / poseEstimatorResult.get().targetsUsed.size(),
                        poseEstimatorResult.get().timestampSeconds,
                        totalAmbiguity / poseEstimatorResult.get().targetsUsed.size()));
            }
        }

        inputs.seenTagIDs = lastSeenTagIDs.stream().mapToInt(Integer::intValue).toArray();
        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    }
}

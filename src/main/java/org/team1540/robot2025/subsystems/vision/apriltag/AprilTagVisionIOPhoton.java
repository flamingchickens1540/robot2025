package org.team1540.robot2025.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.*;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team1540.robot2025.FieldConstants;

public class AprilTagVisionIOPhoton extends AprilTagVisionIO {
    protected final PhotonCamera camera;

    protected final Transform3d cameraTransformMeters;

    public AprilTagVisionIOPhoton(String cameraName, Transform3d cameraTransformMeters) {
        super(cameraName);
        this.camera = new PhotonCamera(cameraName);
        this.cameraTransformMeters = cameraTransformMeters;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        Set<Integer> tagIDs = new HashSet<>();
        List<PoseObservation> poseObservations = new ArrayList<>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (result.getMultiTagResult().isPresent()) {
                MultiTargetPNPResult multitagResult = result.getMultiTagResult().get();
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(cameraTransformMeters.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalDistance = 0.0;
                for (PhotonTrackedTarget target : result.targets) {
                    totalDistance +=
                            target.getBestCameraToTarget().getTranslation().getNorm();
                }

                tagIDs.addAll(multitagResult.fiducialIDsUsed.stream()
                        .map(Short::intValue)
                        .toList());
                poseObservations.add(new PoseObservation(
                        robotPose,
                        multitagResult.fiducialIDsUsed.size(),
                        totalDistance / result.targets.size(),
                        result.getTimestampSeconds(),
                        multitagResult.estimatedPose.ambiguity));
            } else if (!result.targets.isEmpty()) {
                PhotonTrackedTarget target = result.targets.get(0);

                Optional<Pose3d> tagPose = FieldConstants.aprilTagLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                            tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.getBestCameraToTarget();
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(cameraTransformMeters.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    tagIDs.add(target.fiducialId);
                    poseObservations.add(new PoseObservation(
                            robotPose,
                            1,
                            cameraToTarget.getTranslation().getNorm(),
                            result.getTimestampSeconds(),
                            target.poseAmbiguity));
                }
            }

            inputs.seenTagIDs = tagIDs.stream().mapToInt(Integer::intValue).toArray();
            inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
        }
    }
}

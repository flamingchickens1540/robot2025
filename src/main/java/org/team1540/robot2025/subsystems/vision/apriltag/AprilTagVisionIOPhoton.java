package org.team1540.robot2025.subsystems.vision.apriltag;

import static org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVisionConstants.APRIL_TAG_FIELD_LAYOUT;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionIOPhoton implements AprilTagVisionIO {
    protected final PhotonCamera camera;

    protected final Transform3d cameraTransformMeters;

    public AprilTagVisionIOPhoton(String cameraName, Transform3d cameraTransformMeters) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraTransformMeters = cameraTransformMeters;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

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

                poseObservations.add(new PoseObservation(
                        robotPose,
                        multitagResult.fiducialIDsUsed.size(),
                        totalDistance / result.targets.size(),
                        result.getTimestampSeconds(),
                        multitagResult.fiducialIDsUsed.stream()
                                .mapToInt(Short::intValue)
                                .toArray(),
                        multitagResult.estimatedPose.ambiguity));
            } else if (!result.targets.isEmpty()) {
                PhotonTrackedTarget target = result.targets.get(0);

                Optional<Pose3d> tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                            tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.getBestCameraToTarget();
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(cameraTransformMeters.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    poseObservations.add(new PoseObservation(
                            robotPose,
                            1,
                            cameraToTarget.getTranslation().getNorm(),
                            result.getTimestampSeconds(),
                            new int[] {target.fiducialId},
                            target.poseAmbiguity));
                }
            }

            inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
        }
    }

    public String getName() {
        return camera.getName();
    }
}

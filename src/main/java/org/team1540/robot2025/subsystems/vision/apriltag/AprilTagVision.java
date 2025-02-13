package org.team1540.robot2025.subsystems.vision.apriltag;

import static org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.FieldConstants;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVisionIO.PoseObservation;

public class AprilTagVision extends SubsystemBase {
    private final AprilTagVisionIO[] visionIOs;
    private final AprilTagVisionIOInputsAutoLogged[] cameraInputs;

    private final Alert[] disconnectedAlerts;

    private AprilTagVision(AprilTagVisionIO... visionIOs) {
        this.visionIOs = visionIOs;

        this.cameraInputs = new AprilTagVisionIOInputsAutoLogged[visionIOs.length];
        this.disconnectedAlerts = new Alert[visionIOs.length];
        for (int i = 0; i < cameraInputs.length; i++) {
            cameraInputs[i] = new AprilTagVisionIOInputsAutoLogged();
            disconnectedAlerts[i] = new Alert(visionIOs[i].name + " is disconnected.", Alert.AlertType.kWarning);
        }
    }

    public void periodic() {
        for (int i = 0; i < visionIOs.length; i++) {
            visionIOs[i].updateInputs(cameraInputs[i]);
            Logger.processInputs("Vision/" + visionIOs[i].name, cameraInputs[i]);
        }

        RobotState robotState = RobotState.getInstance();

        List<Pose3d> acceptedPoses = new ArrayList<>();
        List<Pose3d> rejectedPoses = new ArrayList<>();
        List<Pose3d> seenTagPoses = new ArrayList<>();
        for (int i = 0; i < visionIOs.length; i++) {
            disconnectedAlerts[i].set(!cameraInputs[i].connected);
            for (PoseObservation poseObservation : cameraInputs[i].poseObservations) {
                if (robotState.addVisionMeasurement(poseObservation)) {
                    acceptedPoses.add(poseObservation.estimatedPoseMeters());
                } else {
                    rejectedPoses.add(poseObservation.estimatedPoseMeters());
                }
            }
            seenTagPoses.addAll(Arrays.stream(cameraInputs[i].seenTagIDs)
                    .mapToObj(tagID ->
                            FieldConstants.aprilTagLayout.getTagPose(tagID).orElse(Pose3d.kZero))
                    .toList());
        }

        Logger.recordOutput("Vision/AcceptedPoses", acceptedPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/RejectedPoses", rejectedPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/SeenTagPoses", seenTagPoses.toArray(new Pose3d[0]));
    }

    public static AprilTagVision createReal() {
        return new AprilTagVision(
                new AprilTagVisionIOPhoton(FL_CAMERA_NAME, FL_CAMERA_TRANSFORM),
                new AprilTagVisionIOPhoton(FR_CAMERA_NAME, FR_CAMERA_TRANSFORM),
                new AprilTagVisionIOPhoton(BL_CAMERA_NAME, BL_CAMERA_TRANSFORM),
                new AprilTagVisionIOPhoton(BR_CAMERA_NAME, BR_CAMERA_TRANSFORM));
    }

    public static AprilTagVision createSim() {
        return new AprilTagVision(
                new AprilTagVisionIOSim(FL_CAMERA_NAME, FL_CAMERA_TRANSFORM),
                new AprilTagVisionIOSim(FR_CAMERA_NAME, FR_CAMERA_TRANSFORM),
                new AprilTagVisionIOSim(BL_CAMERA_NAME, BL_CAMERA_TRANSFORM),
                new AprilTagVisionIOSim(BR_CAMERA_NAME, BR_CAMERA_TRANSFORM));
    }

    public static AprilTagVision createDummy() {
        return new AprilTagVision(
                new AprilTagVisionIO(FL_CAMERA_NAME) {},
                new AprilTagVisionIO(FR_CAMERA_NAME) {},
                new AprilTagVisionIO(BL_CAMERA_NAME) {},
                new AprilTagVisionIO(BR_CAMERA_NAME) {});
    }
}

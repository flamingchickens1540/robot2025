package org.team1540.robot2025.subsystems.vision.apriltag;

import static org.team1540.robot2025.Constants.Vision.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
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
            disconnectedAlerts[i] =
                    new Alert("Oh dear, " + visionIOs[i].getName() + " is disconnected D;", Alert.AlertType.kWarning);
        }
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
        return new AprilTagVision();
    }

    public void periodic() {
        for (int i = 0; i < visionIOs.length; i++) {
            visionIOs[i].updateInputs(cameraInputs[i]);
            Logger.processInputs("Vision/" + visionIOs[i].getName(), cameraInputs[i]);
        }

        RobotState robotState = RobotState.getInstance();

        for (int i = 0; i < visionIOs.length; i++) {
            disconnectedAlerts[i].set(!cameraInputs[i].connected);
            for (PoseObservation poseObservation : cameraInputs[i].poseObservations) {
                robotState.addVisionMeasurement(poseObservation);
            }
        }
    }
}

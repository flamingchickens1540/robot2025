package org.team1540.robot2025.subsystems.vision.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static org.team1540.robot2025.subsystems.vision.coral.CoralVisionConstants.*;

public class CoralVision extends SubsystemBase {
    private final CoralVisionIO io;
    private final CoralVisionIOInputsAutoLogged inputs = new CoralVisionIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    private CoralVision(CoralVisionIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert(io.getName() + " is disconnected oh noes", Alert.AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralVision", inputs);
        disconnectedAlert.set(!inputs.connected);
    }

    public boolean hasCoralDetection() {
        return inputs.hasDetection;
    }

     public ChassisSpeeds getIntakeAssistVelocity(ChassisSpeeds currentVelocity) {
        ChassisSpeeds assistVelocity = new ChassisSpeeds(0,0,0);
        if (inputs.hasDetection) {
            Rotation2d xRotation = inputs.latestDetection.tx();

            assistVelocity = new ChassisSpeeds(
                    currentVelocity.vyMetersPerSecond,
                    -currentVelocity.vxMetersPerSecond,
                    0).times(xRotation.getTan()*TRANSLATION_KP);
        }
        Logger.recordOutput("CoralVision/AssistVelocity", assistVelocity);
        return assistVelocity;
    }

    public static CoralVision createReal() {
        return new CoralVision(new CoralVisionIOLimelight(CAMERA_NAME));
    }

    public static CoralVision createSim() {
        return new CoralVision(new CoralVisionIO() {});
    }

    public static CoralVision createDummy() {
        return new CoralVision(new CoralVisionIO() {});
    }
}

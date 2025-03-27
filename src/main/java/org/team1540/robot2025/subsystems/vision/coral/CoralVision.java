package org.team1540.robot2025.subsystems.vision.coral;

import static org.team1540.robot2025.subsystems.vision.coral.CoralVisionConstants.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.RobotState;

public class CoralVision extends SubsystemBase {
    private final CoralVisionIO io;
    private final CoralVisionIOInputsAutoLogged inputs = new CoralVisionIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    private CoralVision(CoralVisionIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert(io.getName() + " is disconnected.", Alert.AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralVision", inputs);

        if (inputs.hasDetection) RobotState.getInstance().addCoralObservation(inputs.latestObservation);

        Logger.recordOutput("CoralVision/CameraPose", CAMERA_POSE.getTranslation());
        disconnectedAlert.set(!inputs.connected);
    }

    @AutoLogOutput(key = "CoralVision/CoralVector")
    public Rotation3d coralVector() {
        CoralVisionIO.CoralObservation observation = inputs.latestObservation;
        return CAMERA_POSE
                .getRotation()
                .rotateBy(new Rotation3d(
                        0, observation.ty().getRadians(), -observation.tx().getRadians()));
    }

    @AutoLogOutput(key = "CoralVision/CoralTranslation")
    public Translation3d coralTranslation() {
        Rotation3d pointAtCoral =
                //                CAMERA_POSE
                //                .getRotation()
                //                .rotateBy(new Rotation3d(RobotState.getInstance().getRobotRotation()));
                //                coralVector();
                coralVector();
        double z = (CAMERA_POSE.getZ() - Units.inchesToMeters(2.25));
        double y = Math.tan(Math.toRadians(90) - Math.abs(pointAtCoral.getY())) * z;
        double x = Math.tan(pointAtCoral.getZ()) * y;
        return new Translation3d(y, x, z);

        //        Translation3d vector = new Translation3d(pointAtCoral.toVector());
        //        vector = new Translation3d(vector.getZ(), vector.getY(), vector.getX());
        //        return vector;
        //        return vector.times(-(CAMERA_POSE.getZ() - Units.inchesToMeters(2.25)) / vector.getZ());
        //        return CAMERA_POSE.getTranslation().plus(new Translation3d(pointAtCoral.toVector()));
        //        double x = Math.tan(pointAtCoral.getY())*(CAMERA_POSE.getZ() - Units.inchesToMeters(2.25));
        //        double y =
        //        return new Translation2d(, );
        //        return new Translation3d(
        //                        //                        ((CAMERA_POSE.getZ() - Units.inchesToMeters(2.25)) /
        //                        // Math.sin(pointAtCoral.getY()))
        //                        //                                / Math.sin(pointAtCoral.getZ()
        //                        //                        ),
        //                        1, pointAtCoral)
        //                .plus(new Translation3d(
        //                        RobotState.getInstance().getEstimatedPose().getTranslation()));
    }

    @AutoLogOutput(key = "CoralVision/CoralPose")
    public Pose2d coralPose() {

        //        return new Pose3d(RobotState.getInstance().getEstimatedPose())
        //                .transformBy(new Transform3d(CAMERA_POSE.toMatrix()))
        //                .transformBy(new Transform3d(coralTranslation(), Rotation3d.kZero))
        //                .getTranslation();
        return RobotState.getInstance()
                .getEstimatedPose()
                .plus(new Transform2d(
                        CAMERA_POSE
                                .getTranslation()
                                .toTranslation2d()
                                .plus(coralTranslation().toTranslation2d()),
                        Rotation2d.kZero));
        //        return RobotState.getInstance()
        //                .getEstimatedPose()
        //                .plus(new Transform2d(
        //                        ))
        //                                .toTranslation2d(),
        //                        Rotation2d.kZero));
        //        CAMERA_POSE.plus(pointAtCoral.unit().times((CAMERA_POSE.getTranslation().getZ() -
        // Units.inchesToMeters(2.25)) / pointAtCoral.get(2)));
    }

    public boolean hasCoralDetection() {
        return inputs.hasDetection;
    }

    public static CoralVision createReal() {
        return new CoralVision(new CoralVisionIOLimelight(CAMERA_NAME));
    }

    public static CoralVision createDummy() {
        return new CoralVision(new CoralVisionIO() {});
    }
}

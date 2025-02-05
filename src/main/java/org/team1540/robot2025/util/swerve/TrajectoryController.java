package org.team1540.robot2025.util.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TrajectoryController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;

    public TrajectoryController(
            double translationKP,
            double translationKI,
            double translationKD,
            double rotationKP,
            double rotationKI,
            double rotationKD) {
        xController = new PIDController(translationKP, translationKI, translationKD);
        yController = new PIDController(translationKP, translationKI, translationKD);
        headingController = new PIDController(rotationKP, rotationKI, rotationKD);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds calculate(Pose2d currentPose, SwerveSample trajectorySample) {
        var speeds = new ChassisSpeeds(
                trajectorySample.vx
                        + xController.calculate(currentPose.getTranslation().getX(), trajectorySample.x),
                trajectorySample.vy
                        + yController.calculate(currentPose.getTranslation().getY(), trajectorySample.y),
                trajectorySample.omega
                        + headingController.calculate(
                                currentPose.getRotation().getRadians(), trajectorySample.heading));
        return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation());
    }

    public void setTranslationPID(double kP, double kI, double kD) {
        xController.setPID(kP, kI, kD);
        yController.setPID(kP, kI, kD);
    }

    public void setHeadingPID(double kP, double kI, double kD) {
        headingController.setPID(kP, kI, kD);
    }
}

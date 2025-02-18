package org.team1540.robot2025.util.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoAlignController {
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController headingController;

    private Pose2d goal = new Pose2d();
    private boolean goalChanged = false;

    public AutoAlignController(
            double translationKP,
            double translationKI,
            double translationKD,
            double rotationKP,
            double rotationKI,
            double rotationKD,
            double maxVelocityMPS,
            double maxAccelerationMPS2,
            double maxAngularVelocityRadPerSec,
            double maxAngularAccelerationRadPerSec2) {
        xController = new ProfiledPIDController(
                translationKP,
                translationKI,
                translationKD,
                new TrapezoidProfile.Constraints(maxVelocityMPS, maxAccelerationMPS2));
        yController = new ProfiledPIDController(
                translationKP,
                translationKI,
                translationKD,
                new TrapezoidProfile.Constraints(maxVelocityMPS, maxAccelerationMPS2));
        headingController = new ProfiledPIDController(
                rotationKP,
                rotationKI,
                rotationKD,
                new TrapezoidProfile.Constraints(maxAngularVelocityRadPerSec, maxAngularAccelerationRadPerSec2));
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setGoal(Pose2d goal) {
        this.goal = goal;
        goalChanged = true;
    }

    public ChassisSpeeds calculate(Pose2d currentPose, ChassisSpeeds currentVelocity) {
        if (goalChanged) {
            reset(currentPose, currentVelocity);
            goalChanged = false;
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xController.calculate(
                        currentPose.getTranslation().getX(),
                        goal.getTranslation().getX()),
                yController.calculate(
                        currentPose.getTranslation().getY(),
                        goal.getTranslation().getY()),
                headingController.calculate(
                        currentPose.getRotation().getRadians(),
                        goal.getRotation().getRadians()),
                currentPose.getRotation());
    }

    public void reset(Pose2d currentPose, ChassisSpeeds currentVelocity) {
        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(currentVelocity, currentPose.getRotation());
        xController.reset(currentPose.getTranslation().getX(), fieldRelativeSpeeds.vxMetersPerSecond);
        yController.reset(currentPose.getTranslation().getY(), fieldRelativeSpeeds.vyMetersPerSecond);
        headingController.reset(currentPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond);
    }

    public boolean atGoal(double translationToleranceMeters, Rotation2d rotationToleranceRadians) {
        return Math.hypot(xController.getPositionError(), yController.getPositionError()) < translationToleranceMeters
                && Math.abs(headingController.getPositionError()) < rotationToleranceRadians.getRadians();
    }

    public void setTranslationPID(double kP, double kI, double kD) {
        xController.setPID(kP, kI, kD);
        yController.setPID(kP, kI, kD);
    }

    public void setHeadingPID(double kP, double kI, double kD) {
        headingController.setPID(kP, kI, kD);
    }
}

package org.team1540.robot2025.util.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.Supplier;

public class AutoAlignController {
    private final ProfiledPIDController translationController;
    private final ProfiledPIDController headingController;

    private Supplier<Pose2d> goal = () -> Pose2d.kZero;
    private boolean goalChanged = false;

    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private double translationError = 0.0;
    private double headingError = 0.0;

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
        translationController = new ProfiledPIDController(
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
        setGoal(() -> goal);
    }

    public void setGoal(Supplier<Pose2d> goal) {
        this.goal = goal;
        goalChanged = true;
    }

    public ChassisSpeeds calculate(Pose2d currentPose, ChassisSpeeds currentVelocity) {
        Pose2d goalPose = goal.get();

        if (goalChanged) {
            reset(goalPose, currentPose, currentVelocity);
            goalChanged = false;
        }

        translationController.reset(
                lastSetpointTranslation.getDistance(goalPose.getTranslation()),
                translationController.getSetpoint().velocity);
        double distance = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        translationError = distance;
        double translationScalar =
                translationController.calculate(distance, 0.0) + translationController.getSetpoint().velocity;
        Translation2d translationVelocity = new Translation2d(
                translationScalar,
                currentPose.getTranslation().minus(goalPose.getTranslation()).getAngle());
        lastSetpointTranslation = new Pose2d(
                        goalPose.getTranslation(),
                        currentPose
                                .getTranslation()
                                .minus(goalPose.getTranslation())
                                .getAngle())
                .transformBy(new Transform2d(translationController.getSetpoint().position, 0.0, Rotation2d.kZero))
                .getTranslation();

        double headingVelocity = headingController.calculate(
                        currentPose.getRotation().getRadians(),
                        goalPose.getRotation().getRadians())
                + headingController.getSetpoint().velocity;
        headingError =
                Math.abs(currentPose.getRotation().minus(goalPose.getRotation()).getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                translationVelocity.getX(), translationVelocity.getY(), headingVelocity, currentPose.getRotation());
    }

    public void reset(Pose2d goalPose, Pose2d currentPose, ChassisSpeeds currentVelocity) {
        ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(currentVelocity, currentPose.getRotation());
        Translation2d translationVelocity =
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
        translationController.reset(
                currentPose.getTranslation().getDistance(goalPose.getTranslation()),
                Math.min(
                        0.0,
                        -translationVelocity
                                .rotateBy(goalPose.getTranslation()
                                        .minus(currentPose.getTranslation())
                                        .getAngle()
                                        .unaryMinus())
                                .getX()));
        headingController.reset(currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
    }

    public boolean atGoal(double translationToleranceMeters, Rotation2d rotationTolerance) {
        return translationError < translationToleranceMeters && headingError < rotationTolerance.getRadians();
    }

    public void setTranslationPID(double kP, double kI, double kD) {
        translationController.setPID(kP, kI, kD);
    }

    public void setHeadingPID(double kP, double kI, double kD) {
        headingController.setPID(kP, kI, kD);
    }

    public void setTranslationConstraints(double maxVelocityMPS, double maxAccelMPS2) {
        translationController.setConstraints(new TrapezoidProfile.Constraints(maxVelocityMPS, maxAccelMPS2));
    }

    public void setRotationConstraints(double maxVelocityRadPerSec, double maxAccelRadPerSec2) {
        headingController.setConstraints(new TrapezoidProfile.Constraints(maxVelocityRadPerSec, maxAccelRadPerSec2));
    }
}

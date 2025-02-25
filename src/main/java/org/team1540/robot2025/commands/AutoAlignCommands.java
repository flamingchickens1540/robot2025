package org.team1540.robot2025.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.FieldConstants.Reef;
import org.team1540.robot2025.FieldConstants.ReefBranch;
import org.team1540.robot2025.FieldConstants.ReefFace;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class AutoAlignCommands {
    private static final LoggedTunableNumber reefAvoidanceRadiusMeters = new LoggedTunableNumber(
            "AutoAlign/ReefAvoidanceRadiusMeters", Reef.faceLength + DrivetrainConstants.DRIVEBASE_RADIUS + 0.5);
    private static final LoggedTunableNumber reefAvoidanceSpeedDegPerSec =
            new LoggedTunableNumber("AutoAlign/ReefAvoidanceSpeedDegPerSec", 1540);

    private static Pose2d getReefDriveTarget(Pose2d robotPose, Pose2d goalPose) {
        Pose2d reefCenter = AllianceFlipUtil.maybeFlipPose(new Pose2d(Reef.center, Rotation2d.kZero));
        Pose2d goalFromReef = goalPose.relativeTo(reefCenter);
        Pose2d robotFromReef = robotPose.relativeTo(reefCenter);
        Rotation2d angularError = goalFromReef
                .getTranslation()
                .getAngle()
                .minus(robotFromReef.getTranslation().getAngle());
        Logger.recordOutput("AutoAlign/AngularError", angularError);

        // If the robot is within a +/-30 deg angle sweep from the goal or close to the goal, go directly there
        if (Math.abs(angularError.getDegrees()) <= 15) {
            return goalPose;
        }
        // If the robot is within or near the reef avoidance radius, drive around the reef
        if (robotFromReef.getTranslation().getNorm() <= reefAvoidanceRadiusMeters.get() + 0.1) {
            Rotation2d angleFromReefCenter = robotFromReef.getTranslation().getAngle();
            Rotation2d step = Rotation2d.fromDegrees(Math.copySign(
                    reefAvoidanceSpeedDegPerSec.get() * Constants.LOOP_PERIOD_SECS, angularError.getDegrees()));
            return new Pose2d(
                    reefCenter
                            .transformBy(new Transform2d(reefAvoidanceRadiusMeters.get(), 0.0, Rotation2d.kZero))
                            .getTranslation()
                            .rotateAround(reefCenter.getTranslation(), angleFromReefCenter.plus(step)),
                    goalPose.getRotation());
        }
        // Otherwise, drive to nearest tangent point on reef avoidance circle
        double distanceFromCenter = robotFromReef.getTranslation().getNorm();
        Rotation2d tangentOffsetAngle =
                Rotation2d.fromRadians(Math.acos(reefAvoidanceRadiusMeters.get() / distanceFromCenter));
        Translation2d tangentPoint1 = reefCenter
                .transformBy(new Transform2d(reefAvoidanceRadiusMeters.get(), 0.0, Rotation2d.kZero))
                .getTranslation()
                .rotateAround(
                        reefCenter.getTranslation(),
                        robotFromReef.getTranslation().getAngle().plus(tangentOffsetAngle));
        Translation2d tangentPoint2 = reefCenter
                .transformBy(new Transform2d(reefAvoidanceRadiusMeters.get(), 0.0, Rotation2d.kZero))
                .getTranslation()
                .rotateAround(
                        reefCenter.getTranslation(),
                        robotFromReef.getTranslation().getAngle().minus(tangentOffsetAngle));
        if (tangentPoint1.getDistance(goalPose.getTranslation())
                < tangentPoint2.getDistance(goalPose.getTranslation())) {
            return new Pose2d(tangentPoint1, goalPose.getRotation());
        } else {
            return new Pose2d(tangentPoint2, goalPose.getRotation());
        }
    }

    public static Command alignToReefPose(Supplier<Pose2d> pose, Drivetrain drivetrain) {
        return drivetrain.driveToPoseCommand(() -> {
            Pose2d target = getReefDriveTarget(RobotState.getInstance().getEstimatedPose(), pose.get());
            Logger.recordOutput("AutoAlign/DriveTarget", target);
            return target;
        });
    }

    public static Command alignToReefPose(Pose2d pose, Drivetrain drivetrain) {
        return alignToReefPose(() -> pose, drivetrain);
    }

    public static Command alignToBranch(ReefBranch branch, Drivetrain drivetrain) {
        return alignToReefPose(() -> AllianceFlipUtil.maybeFlipPose(branch.scorePosition), drivetrain);
    }

    public static Command alignToNearestBranch(Drivetrain drivetrain) {
        return Commands.defer(() -> alignToReefPose(Reef.closestBranch(), drivetrain), Set.of(drivetrain));
    }

    public static Command alignToNearestFace(Drivetrain drivetrain, BooleanSupplier isRight) {
        return Commands.defer(
                () -> {
                    ReefFace closestFace = Reef.faces.get(0);
                    double closestDistance = Double.MAX_VALUE;
                    boolean flipDirection = false;

                    for (int i = 0; i < Reef.faces.size(); i++) {
                        Pose2d facePose =
                                AllianceFlipUtil.maybeFlipPose(Reef.faces.get(i).pose());
                        double distance = RobotState.getInstance()
                                .getEstimatedPose()
                                .minus(facePose)
                                .getTranslation()
                                .getNorm();
                        if (distance < closestDistance) {
                            closestDistance = distance;
                            closestFace = Reef.faces.get(i);
                            if (i >= 2 && i != 5) flipDirection = true;
                        }
                    }

                    if (isRight.getAsBoolean()) {
                        return alignToReefPose(
                                AllianceFlipUtil.maybeFlipPose(
                                        flipDirection ? closestFace.leftBranchScore() : closestFace.rightBranchScore()),
                                drivetrain);
                    } else {
                        return alignToReefPose(
                                AllianceFlipUtil.maybeFlipPose(
                                        flipDirection ? closestFace.rightBranchScore() : closestFace.leftBranchScore()),
                                drivetrain);
                    }
                },
                Set.of(drivetrain));
    }
}

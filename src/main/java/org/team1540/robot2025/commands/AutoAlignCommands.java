package org.team1540.robot2025.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.FieldConstants.Reef;
import org.team1540.robot2025.FieldConstants.ReefBranch;
import org.team1540.robot2025.FieldConstants.ReefFace;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2025.subsystems.grabber.GrabberConstants;
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class AutoAlignCommands {
    private static final LoggedTunableNumber reefAvoidanceRadiusMeters = new LoggedTunableNumber(
            "AutoAlign/ReefAvoidanceRadiusMeters", Reef.faceLength + DrivetrainConstants.DRIVEBASE_RADIUS + 0.5);
    private static final LoggedTunableNumber reefAvoidanceLookaheadDeg =
            new LoggedTunableNumber("AutoAlign/ReefAvoidanceLookaheadDeg", 20);
    private static final LoggedTunableNumber finalReefAvoidanceLookaheadDeg =
            new LoggedTunableNumber("AutoAlign/FinalReefAvoidanceLookaheadDeg", 8);
    private static final LoggedTunableNumber finalReefAvoidanceSectorDeg =
            new LoggedTunableNumber("AutoAlign/FinalReefAvoidanceSectorDeg", 15);
    private static final LoggedTunableNumber finalAlignLookaheadMeters =
            new LoggedTunableNumber("AutoAlign/FinalAlignLookaheadMeters", 0.05);
    private static final LoggedTunableNumber finalAlignToleranceDeg =
            new LoggedTunableNumber("AutoAlign/FinalAlignToleranceDeg", 10);
    private static final LoggedTunableNumber finalAlignDistanceMeters =
            new LoggedTunableNumber("AutoAlign/FinalAlignDistanceMeters", 0.9);

    private static Pose2d getReefDriveTarget(Pose2d robotPose, Pose2d goalPose) {
        Pose2d reefCenter = AllianceFlipUtil.maybeFlipPose(new Pose2d(Reef.center, Rotation2d.kZero));
        Pose2d goalFromReef = goalPose.relativeTo(reefCenter);
        Pose2d robotFromReef = robotPose.relativeTo(reefCenter);
        Rotation2d goalAngleFromReef = goalFromReef.getTranslation().getAngle();
        Rotation2d angularError =
                goalAngleFromReef.minus(robotFromReef.getTranslation().getAngle());

        // If the robot is within the final align sector of the goal, go directly there
        if (Math.abs(angularError.getDegrees()) <= finalAlignToleranceDeg.get()) {
            double distanceToGoal = robotFromReef.getTranslation().getDistance(goalFromReef.getTranslation());
            if (distanceToGoal <= finalAlignDistanceMeters.get()) {
                Pose2d interpolatedPose =
                        robotPose.interpolate(goalPose, finalAlignLookaheadMeters.get() / distanceToGoal);
                return new Pose2d(interpolatedPose.getTranslation(), goalPose.getRotation());
            }
            return goalPose;
        }
        // If the robot is within or near the reef avoidance radius, drive around the reef
        if (robotFromReef.getTranslation().getNorm() <= reefAvoidanceRadiusMeters.get() + 0.1) {
            Rotation2d angleFromReefCenter = robotFromReef.getTranslation().getAngle();
            Rotation2d lookahead = Rotation2d.fromDegrees(
                    Math.abs(angularError.getDegrees()) >= finalReefAvoidanceSectorDeg.get()
                            ? reefAvoidanceLookaheadDeg.get()
                            : finalReefAvoidanceLookaheadDeg.get());
            Rotation2d step = Rotation2d.fromDegrees(Math.copySign(lookahead.getDegrees(), angularError.getDegrees()));
            Rotation2d nextAngle = Math.abs(angularError.getDegrees()) <= lookahead.getDegrees()
                    ? goalAngleFromReef.minus(Rotation2d.fromDegrees(
                            Math.copySign(finalAlignToleranceDeg.get(), angularError.getDegrees())))
                    : angleFromReefCenter.plus(step);
            return new Pose2d(
                    reefCenter
                            .transformBy(new Transform2d(reefAvoidanceRadiusMeters.get(), 0.0, Rotation2d.kZero))
                            .getTranslation()
                            .rotateAround(reefCenter.getTranslation(), nextAngle),
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

    public static Command alignToReefPose(ReefFace face, Supplier<Pose2d> pose, Drivetrain drivetrain) {
        return drivetrain.driveToPoseCommand(
                () -> {
                    Pose2d alignmentPoseEstimate = RobotState.getInstance().getReefAlignmentPose(face);
                    Pose2d target = getReefDriveTarget(alignmentPoseEstimate, pose.get());

                    Logger.recordOutput("AutoAlign/AlignmentPoseEstimate", alignmentPoseEstimate);
                    Logger.recordOutput("AutoAlign/DriveTarget", target);
                    return target;
                },
                () -> RobotState.getInstance().getReefAlignmentPose(face));
    }

    public static Command alignToReefPose(ReefFace face, Pose2d pose, Drivetrain drivetrain) {
        return alignToReefPose(face, () -> pose, drivetrain);
    }

    public static Command alignToBranch(ReefBranch branch, Drivetrain drivetrain, BooleanSupplier shouldReverse) {
        return alignToReefPose(
                branch.face,
                () -> {
                    if (!shouldReverse.getAsBoolean()) return AllianceFlipUtil.maybeFlipPose(branch.scorePosition);
                    else {
                        Pose2d pose = AllianceFlipUtil.maybeFlipPose(branch.scorePosition);
                        return new Pose2d(
                                        pose.getTranslation(),
                                        pose.getRotation().rotateBy(Rotation2d.k180deg))
                                .transformBy(
                                        new Transform2d(0.0, GrabberConstants.Y_OFFSET_METERS * 2, Rotation2d.kZero));
                    }
                },
                drivetrain);
    }

    public static Command alignToBranch(ReefBranch branch, Drivetrain drivetrain) {
        return alignToBranch(branch, drivetrain, () -> RobotState.getInstance().shouldReverseCoral(branch));
    }

    public static Command alignToBranchNearestSide(ReefBranch branch, Drivetrain drivetrain) {
        return Commands.defer(
                () -> {
                    Pose2d pose = AllianceFlipUtil.maybeFlipPose(branch.scorePosition);
                    if (RobotState.getInstance().shouldReverseCoral(branch))
                        pose = new Pose2d(
                                        pose.getTranslation(),
                                        pose.getRotation().rotateBy(Rotation2d.k180deg))
                                .transformBy(
                                        new Transform2d(0.0, GrabberConstants.Y_OFFSET_METERS * 2, Rotation2d.kZero));
                    Pose2d finalPose = pose;
                    return alignToReefPose(branch.face, () -> finalPose, drivetrain);
                },
                Set.of(drivetrain));
    }

    public static Command alignToNearestBranch(Drivetrain drivetrain) {
        return Commands.defer(() -> alignToBranch(Reef.closestBranch().get(), drivetrain), Set.of(drivetrain));
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

                    Pose2d pose;
                    if (isRight.getAsBoolean()) {
                        pose = AllianceFlipUtil.maybeFlipPose(
                                flipDirection ? closestFace.leftBranchScore() : closestFace.rightBranchScore());
                    } else {
                        pose = AllianceFlipUtil.maybeFlipPose(
                                flipDirection ? closestFace.rightBranchScore() : closestFace.leftBranchScore());
                    }
                    if (Math.abs(pose.getRotation()
                                    .minus(RobotState.getInstance().getRobotRotation())
                                    .getDegrees())
                            >= 90) {
                        pose = new Pose2d(
                                        pose.getTranslation(),
                                        pose.getRotation().rotateBy(Rotation2d.k180deg))
                                .transformBy(
                                        new Transform2d(0.0, GrabberConstants.Y_OFFSET_METERS * 2, Rotation2d.kZero));
                    }

                    return alignToReefPose(closestFace, pose, drivetrain);
                },
                Set.of(drivetrain));
    }

    public static Command alignToDealgifyPose(ReefFace face, Drivetrain drivetrain) {
        return Commands.defer(
                () -> alignToReefPose(face, AllianceFlipUtil.maybeFlipPose(face.dealgifyPosition()), drivetrain),
                Set.of(drivetrain));
    }
}

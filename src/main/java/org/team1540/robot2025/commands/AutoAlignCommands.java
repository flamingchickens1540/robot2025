package org.team1540.robot2025.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team1540.robot2025.FieldConstants;
import org.team1540.robot2025.FieldConstants.ReefBranch;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2025.util.AllianceFlipUtil;

public class AutoAlignCommands {
    public static Command alignToPose(Supplier<Pose2d> pose, Drivetrain drivetrain) {
        return Commands.defer(
                () -> AutoBuilder.pathfindToPose(
                                pose.get(),
                                new PathConstraints(
                                        DrivetrainConstants.MAX_LINEAR_SPEED_MPS * 0.5,
                                        DrivetrainConstants.MAX_LINEAR_ACCEL_MPS2 * 0.5,
                                        DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.5,
                                        DrivetrainConstants.MAX_ANGULAR_ACCEL_RAD_PER_SEC2 * 0.5))
                        .until(() -> RobotState.getInstance()
                                        .getEstimatedPose()
                                        .minus(pose.get())
                                        .getTranslation()
                                        .getNorm()
                                < 0.4)
                        .andThen(drivetrain.alignToPoseCommand(pose.get())),
                Set.of(drivetrain));
    }

    public static Command alignToBranch(ReefBranch branch, Drivetrain drivetrain) {
        return alignToPose(() -> AllianceFlipUtil.maybeFlipPose(branch.scorePosition), drivetrain);
    }

    public static Command alignToNearestBranch(Drivetrain drivetrain) {
        return Commands.defer(
                () -> {
                    Pose2d closestBranch = new Pose2d();
                    double closestDistance = Double.MAX_VALUE;

                    for (Pose2d pose : FieldConstants.Reef.scorePositions) {
                        pose = AllianceFlipUtil.maybeFlipPose(pose);
                        double distance = RobotState.getInstance()
                                .getEstimatedPose()
                                .minus(pose)
                                .getTranslation()
                                .getNorm();
                        if (distance < closestDistance) {
                            closestDistance = distance;
                            closestBranch = pose;
                        }
                    }

                    Pose2d finalClosestBranch = closestBranch;
                    return alignToPose(() -> finalClosestBranch, drivetrain);
                },
                Set.of(drivetrain));
    }

    public static Command alignToNearestFace(Drivetrain drivetrain, BooleanSupplier isRight) {
        return Commands.defer(
                () -> {
                    Pose2d closestBranch = new Pose2d();
                    double closestDistance = Double.MAX_VALUE;

                    for (int i = isRight.getAsBoolean() ? 1 : 0;
                            i < FieldConstants.Reef.scorePositions.size();
                            i += 2) {
                        Pose2d pose = AllianceFlipUtil.maybeFlipPose(FieldConstants.Reef.scorePositions.get(i));
                        double distance = RobotState.getInstance()
                                .getEstimatedPose()
                                .minus(pose)
                                .getTranslation()
                                .getNorm();
                        if (distance < closestDistance) {
                            closestDistance = distance;
                            closestBranch = pose;
                        }
                    }

                    Pose2d finalClosestBranch = closestBranch;
                    return alignToPose(() -> finalClosestBranch, drivetrain);
                },
                Set.of(drivetrain));
    }
}

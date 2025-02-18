package org.team1540.robot2025.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import org.team1540.robot2025.FieldConstants;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2025.util.AllianceFlipUtil;

public class AutoAlignCommands {
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
                    return AutoBuilder.pathfindToPose(
                                    finalClosestBranch,
                                    new PathConstraints(
                                            DrivetrainConstants.MAX_LINEAR_SPEED_MPS * 0.75,
                                            DrivetrainConstants.MAX_LINEAR_ACCEL_MPS2 * 0.75,
                                            DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.75,
                                            DrivetrainConstants.MAX_ANGULAR_ACCEL_RAD_PER_SEC2 * 0.75),
                                    DrivetrainConstants.MAX_LINEAR_SPEED_MPS * 0.5)
                            .until(() -> RobotState.getInstance()
                                            .getEstimatedPose()
                                            .minus(finalClosestBranch)
                                            .getTranslation()
                                            .getNorm()
                                    < 0.4)
                            .andThen(drivetrain.alignToPoseCommand(finalClosestBranch));
                },
                Set.of(drivetrain));
    }
}

package org.team1540.robot2025.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
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
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class AutoAlignCommands {
    private static final LoggedTunableNumber linearSpeedFactor =
            new LoggedTunableNumber("AutoAlign/LinearSpeedFactor", 0.5);
    private static final LoggedTunableNumber linearAccelFactor =
            new LoggedTunableNumber("AutoAlign/LinearAccelFactor", 0.5);
    private static final LoggedTunableNumber rotationSpeedFactor =
            new LoggedTunableNumber("AutoAlign/RotationSpeedFactor", 0.5);
    private static final LoggedTunableNumber rotationAccelFactor =
            new LoggedTunableNumber("AutoAlign/RotationAccelFactor", 0.5);

    public static Command alignToPose(Supplier<Pose2d> pose, Drivetrain drivetrain) {
        return Commands.defer(
                () -> AutoBuilder.pathfindToPose(
                                pose.get(),
                                new PathConstraints(
                                        DrivetrainConstants.MAX_LINEAR_SPEED_MPS * linearSpeedFactor.get(),
                                        DrivetrainConstants.MAX_LINEAR_ACCEL_MPS2 * linearAccelFactor.get(),
                                        DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC * rotationSpeedFactor.get(),
                                        DrivetrainConstants.MAX_ANGULAR_ACCEL_RAD_PER_SEC2 * rotationAccelFactor.get()))
                        .until(() -> RobotState.getInstance()
                                        .getEstimatedPose()
                                        .minus(pose.get())
                                        .getTranslation()
                                        .getNorm()
                                < 0.4)
                        .andThen(drivetrain.alignToPoseCommand(pose.get()))
                        .deadlineFor(Commands.run(() -> Logger.recordOutput("AutoAlign/GoalPose", pose.get()))),
                Set.of(drivetrain));
    }

    public static Command alignToPose(Pose2d pose, Drivetrain drivetrain) {
        return alignToPose(() -> pose, drivetrain);
    }

    public static Command alignToBranch(ReefBranch branch, Drivetrain drivetrain) {
        return alignToPose(() -> AllianceFlipUtil.maybeFlipPose(branch.scorePosition), drivetrain);
    }

    public static Command alignToNearestBranch(Drivetrain drivetrain) {
        return Commands.defer(() -> alignToPose(Reef.closestBranch(), drivetrain), Set.of(drivetrain));
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
                        return alignToPose(
                                AllianceFlipUtil.maybeFlipPose(
                                        flipDirection ? closestFace.leftBranchScore() : closestFace.rightBranchScore()),
                                drivetrain);
                    } else {
                        return alignToPose(
                                AllianceFlipUtil.maybeFlipPose(
                                        flipDirection ? closestFace.rightBranchScore() : closestFace.leftBranchScore()),
                                drivetrain);
                    }
                },
                Set.of(drivetrain));
    }
}

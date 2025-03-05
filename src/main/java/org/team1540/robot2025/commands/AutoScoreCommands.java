package org.team1540.robot2025.commands;

import static org.team1540.robot2025.FieldConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.Superstructure;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class AutoScoreCommands {
    private static final LoggedTunableNumber prepareDistanceMeters =
            new LoggedTunableNumber("AutoScore/PrepareDistanceMeters", 1.0);

    public static Command alignToBranchAndScore(
            ReefBranch branch, ReefHeight height, Drivetrain drivetrain, Superstructure superstructure) {
        return Commands.defer(
                () -> {
                    boolean reverse = RobotState.getInstance().shouldReverseCoral(branch)
                            //                            || height == ReefHeight.L4
                            || height == ReefHeight.L1;
                    return AutoAlignCommands.alignToBranch(branch, drivetrain, () -> reverse)
                            .asProxy()
                            .alongWith(Commands.waitUntil(() -> RobotState.getInstance()
                                                    .getEstimatedPose()
                                                    .getTranslation()
                                                    .getDistance(AllianceFlipUtil.maybeFlipTranslation(
                                                            branch.scorePosition.getTranslation()))
                                            <= prepareDistanceMeters.get())
                                    .andThen(superstructure.scoreCoral(height, () -> reverse)));
                },
                Set.of(superstructure.intake, superstructure.elevator, superstructure.arm, superstructure.grabber));
    }

    public static Command alignToFaceAndDealgify(ReefFace face, Drivetrain drivetrain, Superstructure superstructure) {
        return AutoAlignCommands.alignToDealgifyPose(face, drivetrain)
                .asProxy()
                .alongWith(Commands.waitUntil(() -> RobotState.getInstance()
                                        .getEstimatedPose()
                                        .getTranslation()
                                        .getDistance(AllianceFlipUtil.maybeFlipTranslation(
                                                face.dealgifyPosition().getTranslation()))
                                <= prepareDistanceMeters.get())
                        .andThen(face.highDealgify() ? superstructure.dealgifyHigh() : superstructure.dealgifyLow()));
    }
}

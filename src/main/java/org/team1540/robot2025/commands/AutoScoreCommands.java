package org.team1540.robot2025.commands;

import static org.team1540.robot2025.FieldConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.Superstructure;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class AutoScoreCommands {
    private static final LoggedTunableNumber prepareDistanceMeters =
            new LoggedTunableNumber("AutoScore/PrepareDistanceMeters", 1.0);

    public static Command alignToBranchAndScore(
            ReefBranch branch,
            ReefHeight height,
            BooleanSupplier scoreConfirm,
            boolean canInvert,
            Drivetrain drivetrain,
            Superstructure superstructure) {
        return Commands.either(
                        AutoAlignCommands.alignToBranchNearestSide(branch, drivetrain),
                        AutoAlignCommands.alignToBranch(branch, drivetrain),
                        () -> canInvert && height != ReefHeight.L1)
                .asProxy()
                .alongWith(Commands.waitUntil(() -> RobotState.getInstance()
                                        .getEstimatedPose()
                                        .getTranslation()
                                        .getDistance(AllianceFlipUtil.maybeFlipTranslation(
                                                branch.scorePosition.getTranslation()))
                                <= prepareDistanceMeters.get())
                        .andThen(superstructure.scoreCoral(height, scoreConfirm)));
    }
}

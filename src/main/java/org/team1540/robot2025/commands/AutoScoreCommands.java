package org.team1540.robot2025.commands;

import static org.team1540.robot2025.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.Superstructure;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.grabber.GrabberConstants;
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.LoggedTunableNumber;
import org.team1540.robot2025.util.math.MathUtils;

public class AutoScoreCommands {
    private static final LoggedTunableNumber prepareDistanceMetersCoralLong =
            new LoggedTunableNumber("AutoScore/PrepareDistanceMetersCoralLong", 1.0);
    private static final LoggedTunableNumber prepareDistanceMetersCoralShort =
            new LoggedTunableNumber("AutoScore/PrepareDistanceMetersCoralShort", 0.5);
    private static final LoggedTunableNumber prepareDistanceMetersAlgae =
            new LoggedTunableNumber("AutoScore/PrepareDistanceMetersAlgae", 2.0);

    public static Command alignToBranchAndScore(
            ReefBranch branch, ReefHeight height, Drivetrain drivetrain, Superstructure superstructure) {
        return Commands.defer(
                () -> {
                    boolean reverse = RobotState.getInstance().shouldReverseCoral(branch);
                    return AutoAlignCommands.alignToBranch(branch, drivetrain, () -> reverse)
                            .asProxy()
                            .deadlineFor(Commands.waitUntil(() -> RobotState.getInstance()
                                                    .getEstimatedPose()
                                                    .getTranslation()
                                                    .getDistance(AllianceFlipUtil.maybeFlipTranslation(
                                                            branch.scorePosition.getTranslation()))
                                            <= prepareDistanceMetersCoralLong.get())
                                    .andThen(
                                            superstructure
                                                    .preScoreCoral(height, () -> reverse)
                                                    .asProxy(),
                                            Commands.waitUntil(() -> RobotState.getInstance()
                                                            .getEstimatedPose()
                                                            .getTranslation()
                                                            .getDistance(AllianceFlipUtil.maybeFlipTranslation(
                                                                    branch.scorePosition.getTranslation()))
                                                    <= prepareDistanceMetersCoralLong.get()),
                                            superstructure
                                                    .scoreCoral(height, () -> reverse)
                                                    .asProxy()))
                            .andThen(
                                    superstructure
                                            .scoreCoral(height, () -> reverse)
                                            .asProxy(),
                                    superstructure.score().asProxy());
                },
                Set.of());
    }

    public static Command alignToBranchAndScoreL1Fallback(
            ReefBranch branch,
            ReefHeight height,
            Drivetrain drivetrain,
            Superstructure superstructure,
            BooleanSupplier shouldL1) {
        return Commands.either(
                alignToBranchAndScore(branch, ReefHeight.L1, drivetrain, superstructure),
                alignToBranchAndScore(branch, height, drivetrain, superstructure),
                shouldL1);
    }

    public static Command alignToFaceAndDealgify(ReefFace face, Drivetrain drivetrain, Superstructure superstructure) {
        return Commands.sequence(AutoAlignCommands.alignToDealgifyPose(
                        face, drivetrain, () -> RobotState.getInstance().shouldReverseAlgae(face)))
                .asProxy()
                .alongWith(Commands.waitUntil(() -> RobotState.getInstance()
                                        .getEstimatedPose()
                                        .getTranslation()
                                        .getDistance(AllianceFlipUtil.maybeFlipTranslation(
                                                face.dealgifyPosition().getTranslation()))
                                <= prepareDistanceMetersAlgae.get())
                        .andThen(superstructure.dealgifyStage(face))
                        .asProxy())
                .andThen(superstructure.dealgify(face).asProxy());
    }

    public static Command alignToFaceAndClean(ReefFace face, Drivetrain drivetrain, Superstructure superstructure) {
        return (Commands.sequence(AutoAlignCommands.alignToDealgifyPose(
                                face, drivetrain, () -> RobotState.getInstance().shouldReverseAlgae(face)))
                        .asProxy()
                        .alongWith(Commands.waitUntil(() -> RobotState.getInstance()
                                                .getEstimatedPose()
                                                .getTranslation()
                                                .getDistance(AllianceFlipUtil.maybeFlipTranslation(
                                                        face.dealgifyPosition().getTranslation()))
                                        <= prepareDistanceMetersAlgae.get())
                                .andThen(superstructure.cleanStage(face))
                                .asProxy()))
                .andThen(
                        superstructure.clean(face).asProxy(),
                        drivetrain
                                .driveToPoseCommand(() -> {
                                    Pose2d pose = AllianceFlipUtil.maybeFlipPose(face.dealgifyPosition())
                                            .transformBy(new Transform2d(0.5, 0, Rotation2d.kZero));
                                    if (!RobotState.getInstance().shouldReverseAlgae(face)) return pose;
                                    else {
                                        return new Pose2d(
                                                        pose.getTranslation(),
                                                        pose.getRotation().rotateBy(Rotation2d.k180deg))
                                                .transformBy(new Transform2d(
                                                        0.0, GrabberConstants.Y_OFFSET_METERS * 2, Rotation2d.kZero));
                                    }
                                })
                                .asProxy());
    }

    public static Command alignToBargeAndScore(Drivetrain drivetrain, Superstructure superstructure) {
        return drivetrain
                .driveToPoseCommand(() -> new Pose2d(
                        AllianceFlipUtil.maybeFlipTranslation(
                                        Barge.middleCage.plus(new Translation2d(-Constants.BUMPER_LENGTH_X_METERS, 0)))
                                .getX(),
                        MathUtils.clamp(
                                RobotState.getInstance().getEstimatedPose().getY(),
                                AllianceFlipUtil.maybeFlipTranslation(Barge.rightCage)
                                        .getY(),
                                AllianceFlipUtil.maybeFlipTranslation(Barge.leftCage)
                                        .getY()),
                        AllianceFlipUtil.maybeFlipRotation(Rotation2d.k180deg)))
                .asProxy()
                .andThen(superstructure.net().asProxy());
    }

    public static Command pointToBargeAndScore(
            Drivetrain drivetrain, Superstructure superstructure, XboxController controller) {
        return drivetrain
                .teleopDriveWithHeadingCommand(
                        controller, () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.k180deg), () -> true)
                .asProxy()
                .alongWith(Commands.waitUntil(() -> Math.abs(RobotState.getInstance()
                                        .getRobotRotation()
                                        .minus(AllianceFlipUtil.maybeReverseRotation(Rotation2d.k180deg))
                                        .getDegrees())
                                < 10)
                        .andThen(superstructure.net()));
    }
}

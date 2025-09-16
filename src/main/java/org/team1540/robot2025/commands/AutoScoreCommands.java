package org.team1540.robot2025.commands;

import static org.team1540.robot2025.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.Superstructure;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.LoggedTunableNumber;
import org.team1540.robot2025.util.math.MathUtils;

public class AutoScoreCommands {
    private static final LoggedTunableNumber prepareDistanceMetersCoralLong =
            new LoggedTunableNumber("AutoScore/PrepareDistanceMetersCoralLong", 2.0);
    private static final LoggedTunableNumber prepareDistanceMetersCoralShort =
            new LoggedTunableNumber("AutoScore/PrepareDistanceMetersCoralShort", 0.5);
    private static final LoggedTunableNumber prepareDistanceMetersAlgae =
            new LoggedTunableNumber("AutoScore/PrepareDistanceMetersAlgae", 2.0);

    public static Command alignToBranchAndScore(
            ReefBranch branch, ReefHeight height, Drivetrain drivetrain, Superstructure superstructure) {
        return Commands.defer(
                () -> {
                    boolean reverse = RobotState.getInstance().shouldReverseCoral(branch) || height != ReefHeight.L4;
                    return (AutoAlignCommands.alignToBranch(branch, drivetrain, () -> reverse, height)
                                    .asProxy())
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
                                                    <= prepareDistanceMetersCoralShort.get()),
                                            superstructure
                                                    .scoreCoral(height, () -> reverse)
                                                    .asProxy()
                                                    .unless(() -> !reverse && height == ReefHeight.L4)))
                            .andThen(
                                    superstructure
                                            .scoreCoral(height, () -> reverse)
                                            .asProxy(),
                                    Commands.waitSeconds(0.2).onlyIf(() -> reverse && height != ReefHeight.L4),
                                    //
                                    Commands.waitSeconds(0.25).onlyIf(DriverStation::isAutonomous),
                                    superstructure.score(false).asProxy());
                },
                Set.of());
    }

    public static Command alignToBranchAndScore(
            ReefBranch branch,
            ReefHeight height,
            Drivetrain drivetrain,
            Superstructure superstructure,
            BooleanSupplier shouldScore) {
        return Commands.either(
                alignToBranchAndScore(branch, height, drivetrain, superstructure), Commands.none(), shouldScore);
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
        return (Commands.sequence(AutoAlignCommands.alignToDealgifyPose(
                                face,
                                drivetrain,
                                () -> RobotState.getInstance().shouldReverseAlgae(face),
                                () -> true,
                                () -> Units.inchesToMeters(9)))
                        .asProxy()
                        .alongWith(Commands.waitUntil(() -> RobotState.getInstance()
                                                .getEstimatedPose()
                                                .getTranslation()
                                                .getDistance(AllianceFlipUtil.maybeFlipTranslation(
                                                        face.dealgifyPosition().getTranslation()))
                                        <= prepareDistanceMetersAlgae.get())
                                .andThen(superstructure.dealgify(face))
                                .asProxy()))
                .andThen(
                        AutoAlignCommands.alignToDealgifyPose(
                                        face,
                                        drivetrain,
                                        () -> RobotState.getInstance().shouldReverseAlgae(face),
                                        () -> false,
                                        () -> Units.inchesToMeters(3.5))
                                .asProxy(),
                        Commands.race(
                                Commands.waitUntil(superstructure.grabber::hasAlgae)
                                        .andThen(Commands.waitSeconds(0.5)),
                                Commands.waitSeconds(2)),
                        AutoAlignCommands.alignToDealgifyPose(
                                        face,
                                        drivetrain,
                                        () -> RobotState.getInstance().shouldReverseAlgae(face),
                                        () -> false,
                                        () -> 0.5)
                                .asProxy(),
                        superstructure.stow().asProxy());
    }

    public static Command alignToFaceAndClean(ReefFace face, Drivetrain drivetrain, Superstructure superstructure) {
        return (Commands.sequence(AutoAlignCommands.alignToDealgifyPose(
                                face,
                                drivetrain,
                                () -> RobotState.getInstance().shouldReverseAlgae(face),
                                () -> true,
                                () -> Units.inchesToMeters(9)))
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
                        AutoAlignCommands.alignToDealgifyPose(
                                        face,
                                        drivetrain,
                                        () -> RobotState.getInstance().shouldReverseAlgae(face),
                                        () -> false,
                                        () -> Units.inchesToMeters(4.5))
                                .asProxy(),
                        superstructure
                                .clean(face)
                                .asProxy()
                                .alongWith(Commands.waitSeconds(0.2)
                                        .andThen(AutoAlignCommands.alignToDealgifyPose(
                                                        face,
                                                        drivetrain,
                                                        () -> RobotState.getInstance()
                                                                .shouldReverseAlgae(face),
                                                        () -> false,
                                                        () -> 0.5)
                                                .asProxy())));
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
                .andThen(superstructure.net().asProxy(), superstructure.score().asProxy());
    }

    public static Command alignToProcessorAndScore(Drivetrain drivetrain, Superstructure superstructure) {
        return drivetrain
                .driveToPoseCommand(() -> AllianceFlipUtil.maybeFlipPose(
                        (Processor.centerFace).plus(new Transform2d(new Translation2d(2, 0), Rotation2d.kZero))))
                .asProxy()
                .alongWith(superstructure.processor().asProxy())
                .andThen(
                        drivetrain
                                .driveToPoseCommand(() -> AllianceFlipUtil.maybeFlipPose(Processor.centerFace))
                                .asProxy(),
                        superstructure.score(false),
                        drivetrain
                                .driveToPoseCommand(() -> AllianceFlipUtil.maybeFlipPose((Processor.centerFace)
                                        .plus(new Transform2d(new Translation2d(2, 0), Rotation2d.kZero))))
                                .asProxy());
    }

    public static Command pointToBargeAndScore(
            Drivetrain drivetrain, Superstructure superstructure, XboxController controller) {
        return Commands.sequence(
                Commands.waitUntil(() -> Math.abs(RobotState.getInstance()
                                .getRobotRotation()
                                .minus(AllianceFlipUtil.maybeReverseRotation(Rotation2d.k180deg))
                                .getDegrees())
                        < 10),
                superstructure.net());
    }

    public static Command warmup(Drivetrain drivetrain, Superstructure superstructure) {
        return alignToBranchAndScore(ReefBranch.A, ReefHeight.L4, drivetrain, superstructure)
                .withTimeout(1.0)
                .ignoringDisable(true)
                .onlyIf(DriverStation::isDisabled);
    }
}

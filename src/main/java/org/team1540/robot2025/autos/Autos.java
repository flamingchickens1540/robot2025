package org.team1540.robot2025.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.FieldConstants.ReefBranch;
import org.team1540.robot2025.FieldConstants.ReefHeight;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.commands.AutoScoreCommands;
import org.team1540.robot2025.subsystems.Superstructure;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.util.AllianceFlipUtil;

public class Autos {
    private static final double AUTO_ALIGN_SWITCH_TIME = 0.8;
    private static final double ALIGN_TIMEOUT = 3;
    private static final double INTAKE_DEPLOY_TIME = 2.5;
    private static final double SCORE_WAIT_TIME = 0.5;

    private final RobotState robotState = RobotState.getInstance();

    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;
    private final Superstructure superstructure;

    public Autos(Drivetrain drivetrain, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.superstructure = superstructure;

        autoFactory = new AutoFactory(
                robotState::getEstimatedPose,
                robotState::resetPose,
                drivetrain::followTrajectory,
                true,
                drivetrain,
                (trajectory, starting) -> {
                    if (starting)
                        robotState.setActiveTrajectory(
                                (AllianceFlipUtil.shouldFlip() ? trajectory.flipped() : trajectory).getPoses());
                    else robotState.clearActiveTrajectory();
                });
    }

    private void resetPoseInSim(AutoRoutine routine, AutoTrajectory startingTrajectory) {
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            routine.active().onTrue(Commands.runOnce(() -> {
                robotState.resetPose(startingTrajectory.getInitialPose().orElse(new Pose2d(3, 3, Rotation2d.kZero)));
                SimulatedArena.getInstance().resetFieldForAuto();
            }));
        }
    }

    public AutoRoutine right3PieceLollipop() {
        final String trajName = "Right3PieceLollipop";

        AutoRoutine routine = autoFactory.newRoutine("Right3PieceLollipop");
        AutoTrajectory startToE = routine.trajectory(trajName, 0);
        AutoTrajectory eToRightLPToD = routine.trajectory(trajName, 1);
        AutoTrajectory dToCenterLPtoC = routine.trajectory(trajName, 2);

        resetPoseInSim(routine, startToE);

        routine.active().onTrue(startToE.cmd());
        routine.active().onTrue(superstructure.zeroCommand());
        startToE.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.E, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.stow().asProxy().alongWith(eToRightLPToD.spawnCmd())));
        eToRightLPToD
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(INTAKE_DEPLOY_TIME)
                        .andThen(superstructure.stow()));
        eToRightLPToD
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.D, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.stow().asProxy().alongWith(dToCenterLPtoC.spawnCmd())));
        dToCenterLPtoC
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(INTAKE_DEPLOY_TIME)
                        .andThen(superstructure.stow()));
        dToCenterLPtoC
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.C, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score().asProxy()));
        return routine;
    }

    public AutoRoutine left3PieceLollipop() {
        final String trajName = "Left3PieceLollipop";

        AutoRoutine routine = autoFactory.newRoutine("Left3PieceLollipop");
        AutoTrajectory startToJ = routine.trajectory(trajName, 0);
        AutoTrajectory jToLeftLPtoK = routine.trajectory(trajName, 1);
        AutoTrajectory kToCenterLPtoL = routine.trajectory(trajName, 2);

        resetPoseInSim(routine, startToJ);

        routine.active().onTrue(startToJ.cmd());
        routine.active().onTrue(superstructure.zeroCommand());
        startToJ.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.J, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.stow().asProxy().alongWith(jToLeftLPtoK.spawnCmd())));
        jToLeftLPtoK
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(INTAKE_DEPLOY_TIME)
                        .andThen(superstructure.stow()));
        jToLeftLPtoK
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.K, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.stow().asProxy().alongWith(kToCenterLPtoL.spawnCmd())));
        kToCenterLPtoL
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(INTAKE_DEPLOY_TIME)
                        .andThen(superstructure.stow()));
        kToCenterLPtoL
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.L, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score().asProxy()));
        return routine;
    }

    public AutoRoutine left3PieceSweep() {
        final String trajName = "Left3PieceSweep";

        AutoRoutine routine = autoFactory.newRoutine("Left3PieceSweep");
        AutoTrajectory startToJ = routine.trajectory(trajName, 0);
        AutoTrajectory jToLeftSrcToK = routine.trajectory(trajName, 1);
        AutoTrajectory kToLeftSrcToL = routine.trajectory(trajName, 2);

        resetPoseInSim(routine, startToJ);

        routine.active().onTrue(startToJ.cmd());
        routine.active().onTrue(superstructure.zeroCommand());
        startToJ.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScoreL1Fallback(
                                ReefBranch.J,
                                ReefHeight.L4,
                                drivetrain,
                                superstructure,
                                () -> !superstructure.grabber.hasCoral())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.stow().asProxy().alongWith(jToLeftSrcToK.spawnCmd())));
        jToLeftSrcToK
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(INTAKE_DEPLOY_TIME)
                        .andThen(superstructure.stow()));
        jToLeftSrcToK
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScoreL1Fallback(
                                ReefBranch.K,
                                ReefHeight.L4,
                                drivetrain,
                                superstructure,
                                () -> !superstructure.grabber.hasCoral())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.stow().asProxy().alongWith(kToLeftSrcToL.spawnCmd())));
        kToLeftSrcToL
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(INTAKE_DEPLOY_TIME)
                        .andThen(superstructure.stow()));
        kToLeftSrcToL
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScoreL1Fallback(
                                ReefBranch.L,
                                ReefHeight.L4,
                                drivetrain,
                                superstructure,
                                () -> !superstructure.grabber.hasCoral())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score().asProxy()));
        return routine;
    }

    public AutoRoutine right3PieceSweep() {
        final String trajName = "Right3PieceSweep";

        AutoRoutine routine = autoFactory.newRoutine("Right3PieceSweep");
        AutoTrajectory startToE = routine.trajectory(trajName, 0);
        AutoTrajectory eToRightSrcToD = routine.trajectory(trajName, 1);
        AutoTrajectory dToRightSrcToC = routine.trajectory(trajName, 2);

        resetPoseInSim(routine, startToE);

        routine.active().onTrue(startToE.cmd());
        routine.active().onTrue(superstructure.zeroCommand());
        startToE.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScoreL1Fallback(
                                ReefBranch.E,
                                ReefHeight.L4,
                                drivetrain,
                                superstructure,
                                () -> !superstructure.grabber.hasCoral())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.stow().asProxy().alongWith(eToRightSrcToD.spawnCmd())));
        eToRightSrcToD
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(INTAKE_DEPLOY_TIME)
                        .andThen(superstructure.stow()));
        eToRightSrcToD
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScoreL1Fallback(
                                ReefBranch.D,
                                ReefHeight.L4,
                                drivetrain,
                                superstructure,
                                () -> !superstructure.grabber.hasCoral())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.stow().asProxy().alongWith(dToRightSrcToC.spawnCmd())));
        dToRightSrcToC
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(INTAKE_DEPLOY_TIME)
                        .andThen(superstructure.stow()));
        dToRightSrcToC
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScoreL1Fallback(
                                ReefBranch.C,
                                ReefHeight.L4,
                                drivetrain,
                                superstructure,
                                () -> !superstructure.grabber.hasCoral())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score().asProxy()));
        return routine;
    }

    public AutoRoutine center1Piece() {
        final String trajName = "Center1Piece";

        AutoRoutine routine = autoFactory.newRoutine("Center1Piece");
        AutoTrajectory startToH = routine.trajectory(trajName, 0);

        resetPoseInSim(routine, startToH);
        routine.active().onTrue(startToH.cmd());
        routine.active().onTrue(superstructure.zeroCommand());

        startToH.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScoreL1Fallback(
                                ReefBranch.H,
                                ReefHeight.L4,
                                drivetrain,
                                superstructure,
                                () -> !superstructure.grabber.hasCoral())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy()));
        return routine;
    }

    public AutoRoutine left4PieceSweepReverse() {
        final String trajName = "Left4PieceSweepReverse";

        AutoRoutine routine = autoFactory.newRoutine("Left4PieceSweepReverse");
        AutoTrajectory startToJ = routine.trajectory(trajName, 0);
        AutoTrajectory jToLeftSrcToK = routine.trajectory(trajName, 1);
        AutoTrajectory kToLeftSrcToL = routine.trajectory(trajName, 2);
        AutoTrajectory lToLeftSrcToA = routine.trajectory(trajName, 3);

        resetPoseInSim(routine, startToJ);

        routine.active().onTrue(startToJ.cmd());
        routine.active().onTrue(superstructure.zeroCommand());
        startToJ.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.J, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                Commands.waitSeconds(0.25)
                                        .andThen(superstructure
                                                .coralGroundIntake()
                                                .asProxy())
                                        .alongWith(jToLeftSrcToK.spawnCmd())));
        jToLeftSrcToK
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.K, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.coralGroundIntake().asProxy().alongWith(kToLeftSrcToL.spawnCmd())));
        kToLeftSrcToL
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.L, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.coralGroundIntake().asProxy().alongWith(lToLeftSrcToA.spawnCmd())));
        lToLeftSrcToA
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.A, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score().asProxy()));
        return routine;
    }

    public AutoRoutine left4Piece() {
        final String trajName = "Left4Piece";

        AutoRoutine routine = autoFactory.newRoutine("Left4Piece");
        AutoTrajectory startToJ = routine.trajectory(trajName, 0);
        AutoTrajectory jToLeftSrc = routine.trajectory(trajName, 1);
        AutoTrajectory leftSrcToK = routine.trajectory(trajName, 2);
        AutoTrajectory kToLeftSrc = routine.trajectory(trajName, 3);
        AutoTrajectory leftSrcToL = routine.trajectory(trajName, 4);
        AutoTrajectory lToLeftSrc = routine.trajectory(trajName, 5);
        AutoTrajectory leftSrcToA = routine.trajectory(trajName, 6);

        resetPoseInSim(routine, startToJ);
        routine.active().onTrue(startToJ.cmd());
        //        routine.active().onTrue(superstructure.zeroCommand());

        startToJ.atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.J, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(Commands.parallel(
                                jToLeftSrc.spawnCmd(),
                                Commands.waitSeconds(0.25)
                                        .andThen(superstructure
                                                .coralGroundIntake()
                                                .asProxy()))));

        jToLeftSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(leftSrcToK.spawnCmd());

        leftSrcToK
                .atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.K, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(kToLeftSrc.spawnCmd())));

        kToLeftSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(leftSrcToL.spawnCmd());

        leftSrcToL
                .atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.L, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(lToLeftSrc.spawnCmd())));

        lToLeftSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(leftSrcToA.spawnCmd());

        leftSrcToA
                .atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(
                        ReefBranch.A, ReefHeight.L4, drivetrain, superstructure));
        return routine;
    }

    public AutoRoutine left4PieceSlow() {
        final String trajName = "Left4PieceSlow";

        AutoRoutine routine = autoFactory.newRoutine("Left4PieceSlow");
        AutoTrajectory startToJ = routine.trajectory(trajName, 0);
        AutoTrajectory jToLeftSrc = routine.trajectory(trajName, 2);
        AutoTrajectory leftSrcToK = routine.trajectory(trajName, 3);
        AutoTrajectory kToLeftSrc = routine.trajectory(trajName, 5);
        AutoTrajectory leftSrcToL = routine.trajectory(trajName, 6);
        AutoTrajectory lToLeftSrc = routine.trajectory(trajName, 8);
        AutoTrajectory leftSrcToA = routine.trajectory(trajName, 9);

        resetPoseInSim(routine, startToJ);
        routine.active().onTrue(startToJ.cmd());
        //        routine.active().onTrue(superstructure.zeroCommand());

        startToJ.done()
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.J, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(Commands.parallel(
                                jToLeftSrc.spawnCmd(),
                                Commands.waitSeconds(0.25)
                                        .andThen(superstructure
                                                .coralGroundIntake()
                                                .asProxy()))));

        jToLeftSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(leftSrcToK.spawnCmd());

        leftSrcToK
                .done()
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.K, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(kToLeftSrc.spawnCmd())));

        kToLeftSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(leftSrcToL.spawnCmd());

        leftSrcToL
                .done()
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.L, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(lToLeftSrc.spawnCmd())));

        lToLeftSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(leftSrcToA.spawnCmd());

        leftSrcToA
                .done()
                .onTrue(AutoScoreCommands.alignToBranchAndScore(
                        ReefBranch.A, ReefHeight.L4, drivetrain, superstructure));
        return routine;
    }

    public AutoRoutine left4PieceRizz() {
        final String trajName = "Left4PieceRizz";

        AutoRoutine routine = autoFactory.newRoutine("Left4PieceRizz");
        AutoTrajectory startToI = routine.trajectory(trajName, 0);
        AutoTrajectory iToLeftSrc = routine.trajectory(trajName, 1);
        AutoTrajectory leftSrcToL = routine.trajectory(trajName, 2);
        AutoTrajectory lToLeftSrc = routine.trajectory(trajName, 3);
        AutoTrajectory leftSrcToK = routine.trajectory(trajName, 4);
        AutoTrajectory kToLeftSrc = routine.trajectory(trajName, 5);
        AutoTrajectory leftSrcToJ = routine.trajectory(trajName, 6);

        resetPoseInSim(routine, startToI);
        routine.active().onTrue(startToI.cmd());
        //        routine.active().onTrue(superstructure.zeroCommand());

        startToI.atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.I, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(Commands.parallel(
                                iToLeftSrc.spawnCmd(),
                                Commands.waitSeconds(0.25)
                                        .andThen(superstructure
                                                .coralGroundIntake()
                                                .asProxy()))));

        iToLeftSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(leftSrcToL.spawnCmd());

        leftSrcToL
                .atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.L, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(lToLeftSrc.spawnCmd())));

        lToLeftSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(leftSrcToK.spawnCmd());

        leftSrcToK
                .atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.K, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(kToLeftSrc.spawnCmd())));

        kToLeftSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(leftSrcToJ.spawnCmd());

        leftSrcToJ
                .atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(
                        ReefBranch.I, ReefHeight.L4, drivetrain, superstructure));
        return routine;
    }

    public AutoRoutine left4PieceEyes() {
        final String trajName = "Left4Piece";

        AutoRoutine routine = autoFactory.newRoutine("Left4PieceEyes");
        AutoTrajectory startToJ = routine.trajectory(trajName, 0);
        AutoTrajectory jToLeftSrc = routine.trajectory(trajName, 1);
        AutoTrajectory leftSrcToK = routine.trajectory(trajName, 2);
        AutoTrajectory kToLeftSrc = routine.trajectory(trajName, 3);
        AutoTrajectory leftSrcToL = routine.trajectory(trajName, 4);
        AutoTrajectory lToLeftSrc = routine.trajectory(trajName, 5);
        AutoTrajectory leftSrcToA = routine.trajectory(trajName, 6);

        resetPoseInSim(routine, startToJ);
        routine.active().onTrue(startToJ.cmd());
        routine.active().onTrue(superstructure.zeroCommand());

        startToJ.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.J, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(0.25).alongWith(jToLeftSrc.spawnCmd()),
                                superstructure.coralGroundIntake().asProxy()));

        jToLeftSrc
                .atTimeBeforeEnd(1.5)
                .onTrue(drivetrain
                        .driveToCoral()
                        .until(superstructure.grabber::hasCoral)
                        .withTimeout(1.5)
                        .andThen(leftSrcToK.spawnCmd()));
        leftSrcToK
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.K, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(kToLeftSrc.spawnCmd())));
        kToLeftSrc
                .atTimeBeforeEnd(1.5)
                .onTrue(drivetrain
                        .driveToCoral()
                        .until(superstructure.grabber::hasCoral)
                        .withTimeout(1.5)
                        .andThen(leftSrcToL.spawnCmd()));
        leftSrcToL
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.L, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(lToLeftSrc.spawnCmd())));
        lToLeftSrc
                .atTimeBeforeEnd(1.5)
                .onTrue(drivetrain
                        .driveToCoral()
                        .until(superstructure.grabber::hasCoral)
                        .withTimeout(1.5)
                        .andThen(leftSrcToA.spawnCmd()));
        leftSrcToA
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(
                        ReefBranch.A, ReefHeight.L4, drivetrain, superstructure));
        return routine;
    }

    public AutoRoutine right4PieceSweepReverse() {
        final String trajName = "Right4PieceSweepReverse";

        AutoRoutine routine = autoFactory.newRoutine("Right4PieceSweepReverse");
        AutoTrajectory startToE = routine.trajectory(trajName, 0);
        AutoTrajectory eToLeftSrcToD = routine.trajectory(trajName, 1);
        AutoTrajectory dToLeftSrcToC = routine.trajectory(trajName, 2);
        AutoTrajectory cToLeftSrcToB = routine.trajectory(trajName, 3);

        resetPoseInSim(routine, startToE);

        routine.active().onTrue(startToE.cmd());
        routine.active().onTrue(superstructure.zeroCommand());
        startToE.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.E, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                Commands.waitSeconds(0.25)
                                        .andThen(superstructure
                                                .coralGroundIntake()
                                                .asProxy())
                                        .alongWith(eToLeftSrcToD.spawnCmd())));
        eToLeftSrcToD
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.D, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.coralGroundIntake().asProxy().alongWith(dToLeftSrcToC.spawnCmd())));
        dToLeftSrcToC
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.C, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false).asProxy(),
                                superstructure.coralGroundIntake().asProxy().alongWith(cToLeftSrcToB.spawnCmd())));
        cToLeftSrcToB
                .atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.B, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score().asProxy()));
        return routine;
    }

    public AutoRoutine right4Piece() {
        final String trajName = "Right4Piece";

        AutoRoutine routine = autoFactory.newRoutine("Right4Piece");
        AutoTrajectory startToE = routine.trajectory(trajName, 0);
        AutoTrajectory eToRightSrc = routine.trajectory(trajName, 1);
        AutoTrajectory rightSrcToD = routine.trajectory(trajName, 2);
        AutoTrajectory dToRightSrc = routine.trajectory(trajName, 3);
        AutoTrajectory rightSrcToC = routine.trajectory(trajName, 4);
        AutoTrajectory cToRightSrc = routine.trajectory(trajName, 5);
        AutoTrajectory rightSrcToB = routine.trajectory(trajName, 6);

        resetPoseInSim(routine, startToE);
        routine.active().onTrue(startToE.cmd());
        //        routine.active().onTrue(superstructure.zeroCommand());

        startToE.atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.E, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(Commands.parallel(
                                eToRightSrc.spawnCmd(),
                                Commands.waitSeconds(0.25)
                                        .andThen(superstructure
                                                .coralGroundIntake()
                                                .asProxy()))));

        eToRightSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(rightSrcToD.spawnCmd());

        rightSrcToD
                .atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.D, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(dToRightSrc.spawnCmd())));

        dToRightSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(rightSrcToC.spawnCmd());

        rightSrcToC
                .atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.C, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(superstructure.coralGroundIntake().asProxy().alongWith(cToRightSrc.spawnCmd())));

        cToRightSrc
                .done()
                //                .or(superstructure.intake::hasCoral)
                .onTrue(rightSrcToB.spawnCmd());

        rightSrcToB
                .atTimeBeforeEnd(0.5)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(
                        ReefBranch.B, ReefHeight.L4, drivetrain, superstructure));
        return routine;
    }

    public AutoRoutine center1Piece2Processor() {
        final String trajName = "New Path";

        AutoRoutine routine = autoFactory.newRoutine("New Path");
        AutoTrajectory startToH = routine.trajectory(trajName, 0);
        AutoTrajectory hToProc = routine.trajectory(trajName, 2);
        AutoTrajectory procToE = routine.trajectory(trajName, 3);
        AutoTrajectory eToProc = routine.trajectory(trajName, 4);

        resetPoseInSim(routine, startToH);
        routine.active().onTrue(startToH.cmd());

        startToH.done()
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.H, ReefHeight.L4, drivetrain, superstructure)
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                AutoScoreCommands.alignToFaceAndDealgify(ReefBranch.H.face, drivetrain, superstructure))
                        .andThen(hToProc.spawnCmd()));
        hToProc.done()
                .onTrue(AutoScoreCommands.alignToProcessorAndScore(drivetrain, superstructure)
                        .andThen(procToE.spawnCmd()));
        procToE.done()
                .onTrue(AutoScoreCommands.alignToFaceAndDealgify(ReefBranch.E.face, drivetrain, superstructure)
                        .andThen(eToProc.spawnCmd()));
        eToProc.done().onTrue(AutoScoreCommands.alignToProcessorAndScore(drivetrain, superstructure));
        return routine;
    }
}

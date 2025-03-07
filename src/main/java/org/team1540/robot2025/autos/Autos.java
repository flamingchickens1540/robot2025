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
    private static final double AUTO_ALIGN_SWITCH_TIME = 0.6;
    private static final double ALIGN_TIMEOUT = 3.0;
    private static final double INTAKE_DEPLOY_TIME = 2.0;
    private static final double SCORE_WAIT_TIME = 1.0;

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
                                superstructure.score(false),
                                superstructure.stow().alongWith(eToRightLPToD.spawnCmd())));
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
                                superstructure.score(false),
                                superstructure.stow().alongWith(dToCenterLPtoC.spawnCmd())));
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
                        .andThen(Commands.waitSeconds(SCORE_WAIT_TIME), superstructure.score()));
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
                                superstructure.score(false),
                                superstructure.stow().alongWith(jToLeftLPtoK.spawnCmd())));
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
                                superstructure.score(false),
                                superstructure.stow().alongWith(kToCenterLPtoL.spawnCmd())));
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
                        .andThen(Commands.waitSeconds(SCORE_WAIT_TIME), superstructure.score()));
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
                                () -> !superstructure.grabber.forwardSensorTripped()
                                        && !superstructure.grabber.reverseSensorTripped())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false),
                                superstructure.stow().alongWith(jToLeftSrcToK.spawnCmd())));
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
                                () -> !superstructure.grabber.forwardSensorTripped()
                                        && !superstructure.grabber.reverseSensorTripped())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false),
                                superstructure.stow().alongWith(kToLeftSrcToL.spawnCmd())));
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
                                () -> !superstructure.grabber.forwardSensorTripped()
                                        && !superstructure.grabber.reverseSensorTripped())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(Commands.waitSeconds(SCORE_WAIT_TIME), superstructure.score()));
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
                                () -> !superstructure.grabber.forwardSensorTripped()
                                        && !superstructure.grabber.reverseSensorTripped())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false),
                                superstructure.stow().alongWith(eToRightSrcToD.spawnCmd())));
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
                                () -> !superstructure.grabber.forwardSensorTripped()
                                        && !superstructure.grabber.reverseSensorTripped())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false),
                                superstructure.stow().alongWith(dToRightSrcToC.spawnCmd())));
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
                                () -> !superstructure.grabber.forwardSensorTripped()
                                        && !superstructure.grabber.reverseSensorTripped())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(Commands.waitSeconds(SCORE_WAIT_TIME), superstructure.score()));
        return routine;
    }

    public AutoRoutine center1PieceBarge() {
        final String trajName = "Center1PieceBarge";

        AutoRoutine routine = autoFactory.newRoutine("Center1PieceBarge");
        AutoTrajectory startToH = routine.trajectory(trajName, 0);
        AutoTrajectory hToBarge = routine.trajectory(trajName, 1);

        resetPoseInSim(routine, startToH);
        routine.active().onTrue(startToH.cmd());
        routine.active().onTrue(superstructure.zeroCommand());

        startToH.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScoreL1Fallback(
                                ReefBranch.H,
                                ReefHeight.L4,
                                drivetrain,
                                superstructure,
                                () -> !superstructure.grabber.forwardSensorTripped()
                                        && !superstructure.grabber.reverseSensorTripped())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false),
                                AutoScoreCommands.alignToFaceAndDealgify(ReefBranch.H.face, drivetrain, superstructure)
                                        .withTimeout(ALIGN_TIMEOUT),
                                hToBarge.spawnCmd()));
        hToBarge.done()
                .onTrue(superstructure.net().andThen(Commands.waitSeconds(SCORE_WAIT_TIME), superstructure.score()));

        return routine;
    }

    public AutoRoutine center1PieceProcessor() {
        final String trajName = "Center1PieceProcessor";

        AutoRoutine routine = autoFactory.newRoutine("Center1PieceProcessor");
        AutoTrajectory startToG = routine.trajectory(trajName, 0);
        AutoTrajectory gToProcessor = routine.trajectory(trajName, 1);

        resetPoseInSim(routine, startToG);
        routine.active().onTrue(startToG.cmd());
        routine.active().onTrue(superstructure.zeroCommand());

        startToG.atTimeBeforeEnd(AUTO_ALIGN_SWITCH_TIME)
                .onTrue(AutoScoreCommands.alignToBranchAndScoreL1Fallback(
                                ReefBranch.G,
                                ReefHeight.L4,
                                drivetrain,
                                superstructure,
                                () -> !superstructure.grabber.forwardSensorTripped()
                                        && !superstructure.grabber.reverseSensorTripped())
                        .withTimeout(ALIGN_TIMEOUT)
                        .andThen(
                                Commands.waitSeconds(SCORE_WAIT_TIME),
                                superstructure.score(false),
                                AutoScoreCommands.alignToFaceAndDealgify(ReefBranch.H.face, drivetrain, superstructure)
                                        .withTimeout(ALIGN_TIMEOUT),
                                gToProcessor.spawnCmd()));
        gToProcessor.done().onTrue(Commands.waitSeconds(SCORE_WAIT_TIME).andThen(superstructure.score()));
        gToProcessor.atTime("Processor").onTrue(superstructure.processor());

        return routine;
    }
}

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
        startToE.atTimeBeforeEnd(0.6)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.E, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(
                                superstructure.score(false),
                                superstructure.stow().alongWith(eToRightLPToD.spawnCmd())));
        eToRightLPToD
                .atTime("DeployIntake")
                .onTrue(superstructure.coralGroundIntake().withTimeout(1.5).andThen(superstructure.stow()));
        eToRightLPToD
                .atTimeBeforeEnd(0.6)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.D, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(
                                superstructure.score(false),
                                superstructure.stow().alongWith(dToCenterLPtoC.spawnCmd())));
        dToCenterLPtoC
                .atTime("DeployIntake")
                .onTrue(superstructure.coralGroundIntake().withTimeout(1.5).andThen(superstructure.stow()));
        dToCenterLPtoC
                .atTimeBeforeEnd(0.6)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.C, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(superstructure.score()));
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
        startToJ.atTimeBeforeEnd(0.6)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.J, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(
                                superstructure.score(false),
                                superstructure.stow().alongWith(jToLeftLPtoK.spawnCmd())));
        jToLeftLPtoK
                .atTime("DeployIntake")
                .onTrue(superstructure.coralGroundIntake().withTimeout(1.5).andThen(superstructure.stow()));
        jToLeftLPtoK
                .atTimeBeforeEnd(0.6)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.K, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(
                                superstructure.score(false),
                                superstructure.stow().alongWith(kToCenterLPtoL.spawnCmd())));
        kToCenterLPtoL
                .atTime("DeployIntake")
                .onTrue(superstructure.coralGroundIntake().withTimeout(1.5).andThen(superstructure.stow()));
        kToCenterLPtoL
                .atTimeBeforeEnd(0.6)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.L, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(superstructure.score()));
        return routine;
    }

    public AutoRoutine left3PieceSweep() {
        final String trajName = "Left3PieceSweep";

        AutoRoutine routine = autoFactory.newRoutine("Left3PieceSweep");
        AutoTrajectory startToJ = routine.trajectory(trajName, 0);
        AutoTrajectory jToLeftLPtoK = routine.trajectory(trajName, 1);
        AutoTrajectory kToCenterLPtoL = routine.trajectory(trajName, 2);

        resetPoseInSim(routine, startToJ);

        routine.active().onTrue(startToJ.cmd());
        routine.active().onTrue(superstructure.zeroCommand());
        startToJ.atTimeBeforeEnd(0.6)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.J, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(
                                superstructure.score(false),
                                superstructure.stow().alongWith(jToLeftLPtoK.spawnCmd())));
        jToLeftLPtoK
                .atTime("DeployIntake")
                .onTrue(superstructure.coralGroundIntake().withTimeout(1.5).andThen(superstructure.stow()));
        jToLeftLPtoK
                .atTimeBeforeEnd(0.6)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.K, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(
                                superstructure.score(false),
                                superstructure.stow().alongWith(kToCenterLPtoL.spawnCmd())));
        kToCenterLPtoL
                .atTime("DeployIntake")
                .onTrue(superstructure.coralGroundIntake().withTimeout(1.5).andThen(superstructure.stow()));
        kToCenterLPtoL
                .atTimeBeforeEnd(0.6)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.L, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(superstructure.score()));
        return routine;
    }
}

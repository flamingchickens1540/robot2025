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
import org.team1540.robot2025.subsystems.Superstructure.SuperstructureState;
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
        startToE.atTimeBeforeEnd(0.4)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.E, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(eToRightLPToD.spawnCmd()));
        eToRightLPToD
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(2.5)
                        .andThen(superstructure.commandToState(SuperstructureState.STOW)));
        eToRightLPToD
                .atTimeBeforeEnd(0.4)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(ReefBranch.D, ReefHeight.L4, drivetrain, superstructure)
                        .andThen(dToCenterLPtoC.spawnCmd()));
        dToCenterLPtoC
                .atTime("DeployIntake")
                .onTrue(superstructure
                        .coralGroundIntake()
                        .withTimeout(2.5)
                        .andThen(superstructure.commandToState(SuperstructureState.STOW)));
        dToCenterLPtoC
                .atTimeBeforeEnd(0.4)
                .onTrue(AutoScoreCommands.alignToBranchAndScore(
                        ReefBranch.C, ReefHeight.L4, drivetrain, superstructure));
        return routine;
    }
}

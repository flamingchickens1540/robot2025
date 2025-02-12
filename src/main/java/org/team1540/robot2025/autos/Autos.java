package org.team1540.robot2025.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.RobotState;
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

    public AutoRoutine testAuto() {
        AutoRoutine routine = autoFactory.newRoutine("Test Auto");
        List<AutoTrajectory> trajectories = List.of(
                routine.trajectory("StartLtoI"),
                routine.trajectory("IJtoSrcLL"),
                routine.trajectory("SrcLLtoK"),
                routine.trajectory("KLtoSrcLR"),
                routine.trajectory("SrcLRtoL"),
                routine.trajectory("KLtoSrcLR"),
                routine.trajectory("SrcLRtoA"));
        resetPoseInSim(routine, trajectories.get(0));

        routine.active()
                .onTrue(Commands.sequence(trajectories.stream()
                        .map(traj -> traj.cmd().andThen(Commands.waitSeconds(0.1)))
                        .toArray(Command[]::new)));
        return routine;
    }
}

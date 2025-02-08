package org.team1540.robot2025.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.stream.IntStream;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.arm.ArmConstants;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.elevator.ElevatorConstants;
import org.team1540.robot2025.subsystems.intake.CoralIntake;
import org.team1540.robot2025.util.AllianceFlipUtil;

public class Autos {
    private final RobotState robotState = RobotState.getInstance();

    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Arm arm;
    private final CoralIntake coralIntake;

    public Autos(Drivetrain drivetrain, Elevator elevator, Arm arm, CoralIntake coralIntake) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.arm = arm;
        this.coralIntake = coralIntake;

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

    private Command scoreCommand() {
        return arm.setpointCommand(ArmConstants.ArmState.STOW)
                .onlyIf(() -> elevator.getPosition() < ElevatorConstants.CLEAR_HEIGHT_M)
                .withTimeout(0.25)
                .andThen(Commands.parallel(
                        elevator.setpointCommand(ElevatorConstants.ElevatorState.L4),
                        Commands.waitUntil(() -> elevator.getPosition() > ElevatorConstants.CLEAR_HEIGHT_M)
                                .andThen(arm.setpointCommand(ArmConstants.ArmState.SCORE))));
    }

    private Command stowCommand() {
        return arm.setpointCommand(ArmConstants.ArmState.STOW)
                .withTimeout(0.25)
                .andThen(elevator.setpointCommand(ElevatorConstants.ElevatorState.BASE));
    }

    private Command sourceCommand() {
        return arm.setpointCommand(ArmConstants.ArmState.STOW)
                .withTimeout(0.25)
                .andThen(elevator.setpointCommand(ElevatorConstants.ElevatorState.SOURCE));
    }

    public AutoRoutine leftSide5Piece() {
        int numSegments = 7;
        String pathName = "5Coral";

        AutoRoutine routine = autoFactory.newRoutine("Left Side 5 Piece");
        AutoTrajectory[] trajectories = IntStream.range(0, numSegments)
                .mapToObj(i -> routine.trajectory(pathName, i))
                .toArray(AutoTrajectory[]::new);
        resetPoseInSim(routine, trajectories[0]);

        routine.active()
                .onTrue(stowCommand()
                        .andThen(
                                trajectories[0].cmd().andThen(drivetrain::stop),
                                Commands.runOnce(drivetrain::stop),
                                scoreCommand(),
                                Commands.waitSeconds(0.25),
                                trajectories[1].cmd().alongWith(sourceCommand()),
                                trajectories[2].cmd().andThen(drivetrain::stop),
                                scoreCommand(),
                                Commands.waitSeconds(0.25),
                                trajectories[3].cmd().alongWith(sourceCommand()),
                                trajectories[4].cmd().andThen(drivetrain::stop),
                                scoreCommand(),
                                Commands.waitSeconds(0.25),
                                trajectories[5].cmd().alongWith(sourceCommand()),
                                trajectories[6].cmd().andThen(drivetrain::stop),
                                scoreCommand()));

        return routine;
    }
}

package org.team1540.robot2025.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.elevator.Elevator;
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
}

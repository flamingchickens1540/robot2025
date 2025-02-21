package org.team1540.robot2025.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import org.team1540.robot2025.FieldConstants.ReefHeight;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.autos.AutoSequence.ReefPosition;
import org.team1540.robot2025.autos.AutoSequence.SourcePosition;
import org.team1540.robot2025.autos.AutoSequence.StartingPosition;
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

    public AutoSequence sequence(String name, StartingPosition startingPosition) {
        return new AutoSequence(name, startingPosition, autoFactory);
    }

    public AutoRoutine testAuto() {
        return sequence("Test Auto", StartingPosition.LEFT)
                .withReefSegment(ReefPosition.H, ReefHeight.L4)
                .withSourceSegment(SourcePosition.LEFT_OUTER)
                .withReefSegment(ReefPosition.I, ReefHeight.L4)
                .withSourceSegment(SourcePosition.LEFT_INNER)
                .withReefSegment(ReefPosition.J, ReefHeight.L4)
                .withSourceSegment(SourcePosition.RIGHT_INNER)
                .withReefSegment(ReefPosition.K, ReefHeight.L4)
                .build();
    }
}

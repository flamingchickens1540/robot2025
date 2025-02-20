package org.team1540.robot2025.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.FieldConstants.ReefHeight;
import org.team1540.robot2025.RobotState;

public class AutoSequence {
    private final List<Enum<? extends Position>> positions = new ArrayList<>();
    private final List<ReefHeight> reefHeights = new ArrayList<>();

    public AutoSequence(StartingPosition startingPosition) {
        positions.add(startingPosition);
    }

    public AutoRoutine build(AutoFactory factory, String name, boolean resetPose) {
        AutoRoutine routine = factory.newRoutine(name);
        List<AutoTrajectory> trajectories = new ArrayList<>();
        List<Command> commands = new ArrayList<>();

        for (int i = 0; i < positions.size() - 1; i++) {
            Position startPosition = (Position) positions.get(i);
            Position endPosition = (Position) positions.get(i + 1);

            // Get trajectory name
            StringBuilder trajectoryName = new StringBuilder();
            if (startPosition instanceof ReefPosition) {
                trajectoryName.append(((ReefPosition) startPosition).side);
            } else {
                trajectoryName.append(startPosition.getTrajectoryName());
            }
            trajectoryName.append("to");
            trajectoryName.append(endPosition.getTrajectoryName());

            AutoTrajectory trajectory = routine.trajectory(trajectoryName.toString());
            trajectories.add(trajectory);
            commands.add(trajectory.cmd());

            // TODO reef and intake commands
        }

        Command autoCommand = Commands.sequence(commands.toArray(new Command[0]));

        if (resetPose && !commands.isEmpty()) {
            autoCommand = autoCommand.beforeStarting(() -> {
                if (trajectories.get(0).getInitialPose().isPresent()) {
                    RobotState.getInstance()
                            .resetPose(trajectories.get(0).getInitialPose().get());
                }
            });
        }

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            // Reset simulated game elements
            autoCommand = autoCommand.beforeStarting(
                    Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto()));
        }

        routine.active().onTrue(autoCommand);
        return routine;
    }

    public AutoRoutine build(AutoFactory factory, String name) {
        return build(factory, name, Constants.CURRENT_MODE == Constants.Mode.SIM);
    }

    public AutoSequence withReefSegment(ReefPosition position, ReefHeight height) {
        if (positions.get(positions.size() - 1) instanceof ReefPosition) {
            throw new IllegalStateException("Cannot add reef segment after another reef segment");
        }

        positions.add(position);
        reefHeights.add(height);
        return this;
    }

    public AutoSequence withSourceSegment(SourcePosition position) {
        if (positions.get(positions.size() - 1) instanceof SourcePosition) {
            throw new IllegalStateException("Cannot add source segment after another source segment");
        }

        positions.add(position);
        return this;
    }

    public interface Position {
        String getTrajectoryName();
    }

    public enum StartingPosition implements Position {
        LEFT("StartL"),
        CENTER("StartC"),
        RIGHT("StartR");

        public final String name;

        StartingPosition(String name) {
            this.name = name;
        }

        @Override
        public String getTrajectoryName() {
            return name;
        }
    }

    public enum SourcePosition implements Position {
        LEFT_LEFT("SrcLL"),
        LEFT_RIGHT("SrcLR"),
        RIGHT_LEFT("SrcRL"),
        RIGHT_RIGHT("SrcRR");

        public final String name;

        SourcePosition(String name) {
            this.name = name;
        }

        @Override
        public String getTrajectoryName() {
            return name;
        }
    }

    public enum ReefPosition implements Position {
        A("AB"),
        B("AB"),
        C("CD"),
        D("CD"),
        E("EF"),
        F("EF"),
        G("GH"),
        H("GH"),
        I("IJ"),
        J("IJ"),
        K("KL"),
        L("KL");

        public final String side;

        ReefPosition(String side) {
            this.side = side;
        }

        @Override
        public String getTrajectoryName() {
            return this.name();
        }
    }
}

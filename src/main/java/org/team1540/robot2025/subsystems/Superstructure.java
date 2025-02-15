package org.team1540.robot2025.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.CoralIntake;

public class Superstructure {
    public enum SuperstructureState {}

    private final Elevator elevator;
    private final Arm arm;
    private final CoralIntake coralIntake;
    private final Grabber grabber;

    private SuperstructureState goalState;

    public Superstructure(Elevator elevator, Arm arm, CoralIntake coralIntake, Grabber grabber) {
        this.elevator = elevator;
        this.arm = arm;
        this.coralIntake = coralIntake;
        this.grabber = grabber;
    }

    @AutoLogOutput(key = "Superstructure/GoalState")
    public SuperstructureState getGoalState() {
        return goalState;
    }
}

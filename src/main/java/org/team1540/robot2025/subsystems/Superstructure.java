package org.team1540.robot2025.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.CoralIntake;

public class Superstructure {
    public enum SuperstructureState {
        STOW(Arm.ArmState.STOW, Elevator.ElevatorState.BASE, CoralIntake.CoralIntakeState.STOW),
        INTAKE_GROUND(Arm.ArmState.INTAKE, Elevator.ElevatorState.BASE, CoralIntake.CoralIntakeState.INTAKE),
        INTAKE_FUNNEL(Arm.ArmState.FUNNEL, Elevator.ElevatorState.FUNNEL, CoralIntake.CoralIntakeState.STOW),
        INTAKE_ALGAE(Arm.ArmState.GROUND_ALGAE, Elevator.ElevatorState.GROUND_ALGAE, CoralIntake.CoralIntakeState.STOW),
        L1_FRONT(Arm.ArmState.SCORE_L1, Elevator.ElevatorState.L1, CoralIntake.CoralIntakeState.STOW),
        L2_FRONT(Arm.ArmState.SCORE_L2_L3, Elevator.ElevatorState.L2, CoralIntake.CoralIntakeState.STOW),
        L3_FRONT(Arm.ArmState.SCORE_L2_L3, Elevator.ElevatorState.L3, CoralIntake.CoralIntakeState.STOW),
        L4_FRONT(Arm.ArmState.SCORE_L4, Elevator.ElevatorState.L4, CoralIntake.CoralIntakeState.STOW),
        DEALGIFY_HIGH_FRONT(
                Arm.ArmState.REEF_ALGAE, Elevator.ElevatorState.REEF_ALGAE_HIGH, CoralIntake.CoralIntakeState.STOW),
        SCORE_BARGE_FRONT(Arm.ArmState.BARGE, Elevator.ElevatorState.BARGE, CoralIntake.CoralIntakeState.STOW),
        L1_BACK(Arm.ArmState.SCORE_L1_REVERSE, Elevator.ElevatorState.L1, CoralIntake.CoralIntakeState.STOW),
        L2_BACK(Arm.ArmState.SCORE_L2_L3_REVERSE, Elevator.ElevatorState.L2, CoralIntake.CoralIntakeState.STOW),
        L3_BACK(Arm.ArmState.SCORE_L2_L3_REVERSE, Elevator.ElevatorState.L3, CoralIntake.CoralIntakeState.STOW),
        L4_BACK(Arm.ArmState.SCORE_L4_REVERSE, Elevator.ElevatorState.L4, CoralIntake.CoralIntakeState.STOW),
        DEALGIFY_LOW_BACK(Arm.ArmState.REEF_ALGAE_REVERSE, Elevator.ElevatorState.REEF_ALGAE_LOW, CoralIntake.CoralIntakeState.STOW),
        DEALGIFY_HIGH_BACK(Arm.ArmState.REEF_ALGAE_REVERSE, Elevator.ElevatorState.REEF_ALGAE_HIGH, CoralIntake.CoralIntakeState.STOW),
        SCORE_BARGE_BACK(Arm.ArmState.BARGE_REVERSE, Elevator.ElevatorState.BARGE, CoralIntake.CoralIntakeState.STOW),
        PROCESSOR(Arm.ArmState.PROCESSOR, Elevator.ElevatorState.PROCESSOR, CoralIntake.CoralIntakeState.STOW)
    ;

        private Arm.ArmState armState;
        private Elevator.ElevatorState elevatorState;
        private CoralIntake.CoralIntakeState intakeState;

        SuperstructureState(
                Arm.ArmState armState, Elevator.ElevatorState elevatorState, CoralIntake.CoralIntakeState intakeState) {
            this.armState = armState;
            this.elevatorState = elevatorState;
            this.intakeState = intakeState;
        }
    }

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

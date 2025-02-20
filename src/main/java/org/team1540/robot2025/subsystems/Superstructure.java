package org.team1540.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.CoralIntake;

public class Superstructure {
    public enum SuperstructureState {
        STOW(Arm.ArmState.STOW, Elevator.ElevatorState.STOW, CoralIntake.CoralIntakeState.STOW),
        STOW_ALGAE(Arm.ArmState.STOW_ALGAE, Elevator.ElevatorState.STOW_ALGAE, CoralIntake.CoralIntakeState.STOW),
        INTAKE_GROUND(Arm.ArmState.INTAKE, Elevator.ElevatorState.STOW, CoralIntake.CoralIntakeState.INTAKE),
        INTAKE_FUNNEL(Arm.ArmState.FUNNEL, Elevator.ElevatorState.FUNNEL, CoralIntake.CoralIntakeState.STOW),
        INTAKE_ALGAE(Arm.ArmState.GROUND_ALGAE, Elevator.ElevatorState.GROUND_ALGAE, CoralIntake.CoralIntakeState.STOW),

        L1_FRONT(Arm.ArmState.SCORE_L1_FRONT, Elevator.ElevatorState.L1_FRONT, CoralIntake.CoralIntakeState.STOW),
        L1_BACK(Arm.ArmState.SCORE_L1_BACK, Elevator.ElevatorState.L1_BACK, CoralIntake.CoralIntakeState.STOW),

        L2_FRONT(Arm.ArmState.SCORE_L2_L3_FRONT, Elevator.ElevatorState.L2, CoralIntake.CoralIntakeState.STOW),
        L2_BACK(Arm.ArmState.SCORE_L2_L3_BACK, Elevator.ElevatorState.L2, CoralIntake.CoralIntakeState.STOW),

        L3_FRONT(Arm.ArmState.SCORE_L2_L3_FRONT, Elevator.ElevatorState.L3, CoralIntake.CoralIntakeState.STOW),
        L3_BACK(Arm.ArmState.SCORE_L2_L3_BACK, Elevator.ElevatorState.L3, CoralIntake.CoralIntakeState.STOW),

        L4_FRONT(Arm.ArmState.SCORE_L4_FRONT, Elevator.ElevatorState.L4, CoralIntake.CoralIntakeState.STOW),
        L4_BACK(Arm.ArmState.SCORE_L4_BACK, Elevator.ElevatorState.L4, CoralIntake.CoralIntakeState.STOW),

        // No dealgify low front
        DEALGIFY_LOW_BACK(
                Arm.ArmState.REEF_ALGAE_BACK, Elevator.ElevatorState.REEF_ALGAE_LOW, CoralIntake.CoralIntakeState.STOW),

        DEALGIFY_HIGH_FRONT(
                Arm.ArmState.REEF_ALGAE_FRONT,
                Elevator.ElevatorState.REEF_ALGAE_HIGH,
                CoralIntake.CoralIntakeState.STOW),
        DEALGIFY_HIGH_BACK(
                Arm.ArmState.REEF_ALGAE_BACK,
                Elevator.ElevatorState.REEF_ALGAE_HIGH,
                CoralIntake.CoralIntakeState.STOW),

        // barge is same from both sides
        SCORE_BARGE_FRONT(
                Arm.ArmState.SCORE_BARGE_FRONT, Elevator.ElevatorState.BARGE, CoralIntake.CoralIntakeState.STOW),
        SCORE_BARGE_BACK(
                Arm.ArmState.SCORE_BARGE_BACK, Elevator.ElevatorState.BARGE, CoralIntake.CoralIntakeState.STOW),

        // no processor front (?)
        PROCESSOR_BACK(Arm.ArmState.PROCESSOR, Elevator.ElevatorState.PROCESSOR, CoralIntake.CoralIntakeState.STOW);

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
    private final double funnelHeight = 1.0;

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

    public Command commandToState(SuperstructureState goalState) {
        this.goalState = goalState;
        if (goalState == SuperstructureState.STOW || goalState == SuperstructureState.STOW_ALGAE) {
            return Commands.sequence(
                    arm.commandToSetpoint(goalState.armState),
                    Commands.parallel(
                            elevator.commandToSetpoint(goalState.elevatorState),
                            coralIntake.commandToSetpoint(goalState.intakeState)));
        } else if (goalState == SuperstructureState.INTAKE_ALGAE || goalState == SuperstructureState.INTAKE_GROUND) {
            return Commands.sequence(
                    arm.commandToSetpoint(Arm.ArmState.STOW),
                    Commands.parallel(
                            elevator.commandToSetpoint(goalState.elevatorState),
                            coralIntake.commandToSetpoint(goalState.intakeState),
                            Commands.waitUntil(this::isArmClear).andThen(arm.commandToSetpoint(goalState.armState))));
        } else {
            return Commands.parallel(
                    elevator.commandToSetpoint(goalState.elevatorState),
                    coralIntake.commandToSetpoint(goalState.intakeState),
                    Commands.waitUntil(this::isArmClear).andThen(arm.commandToSetpoint(goalState.armState)));
        }
    }
    // TODO: Simon's fun thing where you can start moving arm early

    public boolean isArmClear() {
        return elevator.getPosition() > funnelHeight;
    }
}

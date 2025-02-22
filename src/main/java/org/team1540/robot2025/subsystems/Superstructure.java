package org.team1540.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.CoralIntake;

public class Superstructure {
    public enum SuperstructureState {
        STOW(Arm.ArmState.STOW, Elevator.ElevatorState.STOW, CoralIntake.CoralIntakeState.STOW),
        STOW_ALGAE(Arm.ArmState.STOW_ALGAE, Elevator.ElevatorState.STOW_ALGAE, CoralIntake.CoralIntakeState.STOW),
        INTAKE_GROUND(Arm.ArmState.INTAKE, Elevator.ElevatorState.GROUND_CORAL, CoralIntake.CoralIntakeState.INTAKE),
        INTAKE_FUNNEL(Arm.ArmState.FUNNEL, Elevator.ElevatorState.FUNNEL, CoralIntake.CoralIntakeState.STOW),
        INTAKE_ALGAE(Arm.ArmState.GROUND_ALGAE, Elevator.ElevatorState.GROUND_ALGAE, CoralIntake.CoralIntakeState.STOW),

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
        return Commands.defer(
                () -> {
                    this.goalState = goalState;
                    if (goalState == SuperstructureState.STOW || goalState == SuperstructureState.STOW_ALGAE) {
                        return Commands.sequence(
                                commandStowArm(),
                                Commands.parallel(
                                        elevator.commandToSetpoint(goalState.elevatorState),
                                        coralIntake.commandToSetpoint(goalState.intakeState)));
                    } else if (goalState == SuperstructureState.INTAKE_GROUND) {
                        return Commands.sequence(
                                commandStowArm(),
                                Commands.parallel(
                                        elevator.commandToSetpoint(goalState.elevatorState),
                                        coralIntake.commandToSetpoint(goalState.intakeState),
                                        Commands.waitUntil(() -> coralIntake
                                                                .getPivotPosition()
                                                                .getDegrees()
                                                        < 30)
                                                .andThen(arm.commandToSetpoint(goalState.armState))));
                    } else {
                        return Commands.sequence(
                                commandStowArm(),
                                Commands.parallel(
                                        elevator.commandToSetpoint(goalState.elevatorState),
                                        coralIntake.commandToSetpoint(goalState.intakeState),
                                        Commands.waitUntil(this::isArmClear)
                                                .onlyIf(() -> goalState
                                                                .armState
                                                                .position()
                                                                .getDegrees()
                                                        < 100)
                                                .andThen(arm.commandToSetpoint(goalState.armState))));
                    }
                },
                Set.of(arm, elevator, coralIntake));
    }
    // TODO: Simon's fun thing where you can start moving arm early

    @AutoLogOutput(key = "Superstructure/ArmClear")
    public boolean isArmClear() {
        return elevator.getPosition() > funnelHeight;
    }

    public Command commandStowArm() {
        return new ConditionalCommand(
                arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE),
                arm.commandToSetpoint(Arm.ArmState.STOW),
                grabber::hasAlgae);
    }

    public Command scoreCoral(SuperstructureState superstructureState, double grabberPower, BooleanSupplier confirm) {
        return Commands.sequence(
                        commandToState(superstructureState),
                        Commands.waitUntil(confirm),
                        grabber.commandRun(grabberPower).until(() -> !grabber.hasCoral()),
                        grabber.commandRun(grabberPower).withTimeout(0.1),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::hasAlgae);
    }

    public Command L2(BooleanSupplier confirm) {
        return scoreCoral(SuperstructureState.L2_BACK, 0.5, confirm);
    }

    public Command L3(BooleanSupplier confirm) {
        return scoreCoral(SuperstructureState.L3_BACK, 0.5, confirm);
    }

    public Command L4(BooleanSupplier confirm) {
        return scoreCoral(SuperstructureState.L4_BACK, 0.5, confirm);
    }

    public Command L2Front(BooleanSupplier confirm) {
        return scoreCoral(SuperstructureState.L2_FRONT, 0.5, confirm);
    }

    public Command L3Front(BooleanSupplier confirm) {
        return scoreCoral(SuperstructureState.L3_FRONT, 0.5, confirm);
    }

    public Command L4Front(BooleanSupplier confirm) {
        return scoreCoral(SuperstructureState.L4_FRONT, 0.5, confirm);
    }

    public Command L1(BooleanSupplier confirm) {
        return scoreCoral(SuperstructureState.L1_BACK, -0.2, confirm);
    }

    public Command dealgifyLow() {
        return Commands.sequence(
                        commandToState(SuperstructureState.DEALGIFY_LOW_BACK),
                        Commands.runOnce(() -> grabber.setPercent(0.25)),
                        Commands.waitUntil(grabber::hasAlgae),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::hasCoral);
    }

    public Command dealgifyHigh() {
        return Commands.sequence(
                        commandToState(SuperstructureState.DEALGIFY_HIGH_BACK),
                        Commands.runOnce(() -> grabber.setPercent(0.25)),
                        Commands.waitUntil(grabber::hasAlgae),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::hasCoral);
    }

    public Command coralGroundIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_GROUND),
                        grabber.commandRun(0.3).alongWith(coralIntake.commandRunRollerFunnel(0.5, 0.5)),
                        commandToState(SuperstructureState.STOW).alongWith(grabber.centerCoral()))
                .unless(grabber::hasAlgae);
    }

    public Command sourceIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_FUNNEL),
                        grabber.commandRun(0.3).until(grabber::hasCoral),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::hasAlgae);
    }

    public Command algaeIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_ALGAE),
                        Commands.runOnce(() -> grabber.setPercent(0.25)),
                        Commands.waitUntil(grabber::hasAlgae),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::hasCoral);
    }

    public Command processor(BooleanSupplier confirm) {
        return Commands.sequence(
                        commandToState(SuperstructureState.PROCESSOR_BACK),
                        Commands.waitUntil(confirm),
                        grabber.commandRun(-0.5).withTimeout(0.5),
                        commandToState(SuperstructureState.STOW))
                .onlyIf(grabber::hasAlgae);
    }

    public Command net() {
        return Commands.sequence(
                elevator.commandToSetpoint(Elevator.ElevatorState.L3),
                arm.commandToSetpoint(Arm.ArmState.REEF_ALGAE_FRONT),
                Commands.parallel(
                        elevator.commandToSetpoint(Elevator.ElevatorState.BARGE),
                        arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE),
                        grabber.commandRun(-0.5).withTimeout(1)),
                commandToState(SuperstructureState.STOW));
        //                .onlyIf(grabber::hasAlgae);
    }

    public Command netReverse() {
        return Commands.sequence(
                arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE),
                Commands.parallel(
                        elevator.commandToSetpoint(Elevator.ElevatorState.BARGE),
                        Commands.parallel(
                                        arm.commandToSetpoint(Arm.ArmState.SCORE_L4_BACK),
                                        grabber.commandRun(-0.5).withTimeout(1))
                                .beforeStarting(Commands.waitSeconds(0.2))
                                .beforeStarting(Commands.waitUntil(() ->
                                        elevator.getPosition() > Elevator.ElevatorState.L3.height.getAsDouble()))),
                commandToState(SuperstructureState.STOW));
        //                .onlyIf(grabber::hasAlgae);
    }

    public Command zeroCommand() {
        return Commands.sequence(
                arm.commandToSetpoint(Arm.ArmState.STOW),
                Commands.parallel(elevator.zeroCommand(), coralIntake.zeroCommand()));
    }
}

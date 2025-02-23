package org.team1540.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.arm.Arm.ArmState;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.elevator.Elevator.ElevatorState;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.CoralIntake;
import org.team1540.robot2025.subsystems.intake.CoralIntake.CoralIntakeState;

public class Superstructure {
    public enum SuperstructureState {
        STOW(ArmState.STOW, ElevatorState.STOW, CoralIntakeState.STOW),
        STOW_ALGAE(ArmState.STOW_ALGAE, ElevatorState.STOW_ALGAE, CoralIntakeState.STOW),
        INTAKE_GROUND(ArmState.INTAKE, ElevatorState.GROUND_CORAL, CoralIntakeState.INTAKE),
        INTAKE_FUNNEL(ArmState.FUNNEL, ElevatorState.FUNNEL, CoralIntakeState.STOW),
        INTAKE_ALGAE(ArmState.GROUND_ALGAE, ElevatorState.GROUND_ALGAE, CoralIntakeState.STOW),

        L1_BACK(ArmState.SCORE_L1_BACK, ElevatorState.L1_BACK, CoralIntakeState.STOW),

        L2_FRONT(ArmState.SCORE_L2_L3_FRONT, ElevatorState.L2, CoralIntakeState.STOW),
        L2_BACK(ArmState.SCORE_L2_L3_BACK, ElevatorState.L2, CoralIntakeState.STOW),

        L3_FRONT(ArmState.SCORE_L2_L3_FRONT, ElevatorState.L3, CoralIntakeState.STOW),
        L3_BACK(ArmState.SCORE_L2_L3_BACK, ElevatorState.L3, CoralIntakeState.STOW),

        L4_FRONT(ArmState.SCORE_L4_FRONT, ElevatorState.L4, CoralIntakeState.STOW),
        L4_BACK(ArmState.SCORE_L4_BACK, ElevatorState.L4, CoralIntakeState.STOW),

        // No dealgify low front
        DEALGIFY_LOW_BACK(ArmState.REEF_ALGAE_BACK, ElevatorState.REEF_ALGAE_LOW, CoralIntakeState.STOW),

        DEALGIFY_HIGH_FRONT(ArmState.REEF_ALGAE_FRONT, ElevatorState.REEF_ALGAE_HIGH, CoralIntakeState.STOW),
        DEALGIFY_HIGH_BACK(ArmState.REEF_ALGAE_BACK, ElevatorState.REEF_ALGAE_HIGH, CoralIntakeState.STOW),

        // barge is same from both sides
        SCORE_BARGE_FRONT(ArmState.SCORE_BARGE_FRONT, ElevatorState.BARGE, CoralIntakeState.STOW),
        SCORE_BARGE_BACK(ArmState.SCORE_BARGE_BACK, ElevatorState.BARGE, CoralIntakeState.STOW),

        // no processor front (?)
        PROCESSOR_BACK(ArmState.PROCESSOR, ElevatorState.PROCESSOR, CoralIntakeState.STOW);

        public final ArmState armState;
        public final ElevatorState elevatorState;
        public final CoralIntakeState intakeState;

        SuperstructureState(ArmState armState, ElevatorState elevatorState, CoralIntakeState intakeState) {
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
                                                                < 30
                                                        && elevator.isAtSetpoint())
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
                arm.commandToSetpoint(ArmState.STOW_ALGAE), arm.commandToSetpoint(ArmState.STOW), grabber::hasAlgae);
    }

    public Command scoreCoral(SuperstructureState superstructureState, double grabberPower, BooleanSupplier confirm) {
        return Commands.sequence(
                        commandToState(superstructureState),
                        Commands.waitUntil(confirm),
                        grabber.commandRun(grabberPower).until(() -> !grabber.reverseSensorTripped()),
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
                .unless(grabber::reverseSensorTripped)
                .handleInterrupt(grabber::stop);
    }

    public Command dealgifyHigh() {
        return Commands.sequence(
                        commandToState(SuperstructureState.DEALGIFY_HIGH_BACK),
                        Commands.runOnce(() -> grabber.setPercent(0.25)),
                        Commands.waitUntil(grabber::hasAlgae),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::reverseSensorTripped)
                .handleInterrupt(grabber::stop);
    }

    public Command coralGroundIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_GROUND),
                        grabber.commandRun(0.3)
                                .until(grabber::forwardSensorTripped)
                                .andThen(grabber.commandRun(0.1).until(grabber::reverseSensorTripped))
                                .deadlineFor(coralIntake.commandRunRollerFunnel(0.5, 0.5)),
                        commandToState(SuperstructureState.STOW).alongWith(grabber.commandRun(0.0)))
                .unless(grabber::hasAlgae);
    }

    public Command coralIntakeEject() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_GROUND),
                        grabber.commandRun(-0.3)
                                .alongWith(coralIntake.commandRunRollerFunnel(-0.5, -0.5)))
                .unless(grabber::hasAlgae);
    }

    public Command sourceIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_FUNNEL),
                        grabber.commandRun(0.3).until(grabber::reverseSensorTripped),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::hasAlgae);
    }

    public Command algaeIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_ALGAE),
                        Commands.runOnce(() -> grabber.setPercent(0.25)),
                        Commands.waitUntil(grabber::hasAlgae),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::reverseSensorTripped)
                .handleInterrupt(grabber::stop);
    }

    public Command processor(BooleanSupplier confirm) {
        return Commands.sequence(
                commandToState(SuperstructureState.PROCESSOR_BACK),
                Commands.waitUntil(confirm),
                grabber.commandRun(-0.5).withTimeout(0.5),
                commandToState(SuperstructureState.STOW));
        //                .onlyIf(grabber::hasAlgae);
    }

    public Command net() {
        return Commands.sequence(
                elevator.commandToSetpoint(ElevatorState.L4),
                arm.commandToSetpoint(ArmState.INTAKE),
                Commands.parallel(
                        elevator.commandToSetpoint(ElevatorState.BARGE),
                        arm.commandToSetpoint(ArmState.STOW_ALGAE),
                        grabber.commandRun(-0.5).withTimeout(1)),
                commandToState(SuperstructureState.STOW));
        //                .onlyIf(grabber::hasAlgae);
    }

    public Command netReverse() {
        return Commands.sequence(
                arm.commandToSetpoint(ArmState.STOW_ALGAE),
                Commands.parallel(
                        elevator.commandToSetpoint(ElevatorState.BARGE),
                        Commands.parallel(
                                        arm.commandToSetpoint(ArmState.SCORE_L4_BACK),
                                        grabber.commandRun(-0.5).withTimeout(1))
                                .beforeStarting(Commands.waitSeconds(0.2))
                                .beforeStarting(Commands.waitUntil(
                                        () -> elevator.getPosition() > ElevatorState.L3.height.getAsDouble()))),
                commandToState(SuperstructureState.STOW));
        //                .onlyIf(grabber::hasAlgae);
    }

    public Command zeroCommand() {
        return Commands.sequence(
                arm.commandToSetpoint(ArmState.STOW),
                Commands.parallel(elevator.zeroCommand(), coralIntake.zeroCommand()));
    }
}

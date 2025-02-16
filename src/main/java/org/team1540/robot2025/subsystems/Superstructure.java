package org.team1540.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.CoralIntake;

public class Superstructure {
    private static boolean hasInstance = false;

    public enum SuperstructureState {}

    private final Elevator elevator;
    private final Arm arm;
    private final CoralIntake coralIntake;
    private final Grabber grabber;

    private SuperstructureState goalState;

    public Superstructure(Elevator elevator, Arm arm, CoralIntake coralIntake, Grabber grabber) {
        if (hasInstance) throw new IllegalStateException("Instance of arm already exists");
        hasInstance = true;
        this.elevator = elevator;
        this.arm = arm;
        this.coralIntake = coralIntake;
        this.grabber = grabber;
    }

    @AutoLogOutput(key = "Superstructure/GoalState")
    public SuperstructureState getGoalState() {
        return goalState;
    }

    public Command stow() {
        return Commands.either(
                Commands.sequence(
                        arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE),
                        elevator.commandToSetpoint(Elevator.ElevatorState.STOW_ALGAE),
                        coralIntake.commandToSetpoint(CoralIntake.CoralIntakeState.STOW)),
                Commands.sequence(
                        arm.commandToSetpoint(Arm.ArmState.STOW),
                        elevator.commandToSetpoint(Elevator.ElevatorState.BASE),
                        coralIntake.commandToSetpoint(CoralIntake.CoralIntakeState.STOW)),
                grabber::hasAlgae);
    }

    public Command scoreCoral(
            Elevator.ElevatorState elevatorPosition,
            Arm.ArmState armPosition,
            double grabberPower,
            BooleanSupplier confirm) {
        return Commands.sequence(
                        arm.commandToSetpoint(Arm.ArmState.STOW),
                        elevator.commandToSetpoint(elevatorPosition),
                        arm.commandToSetpoint(armPosition),
                        Commands.waitUntil(confirm),
                        grabber.commandRun(grabberPower).until(() -> !grabber.hasCoral()),
                        grabber.commandRun(grabberPower).withTimeout(0.1),
                        stow())
                .unless(grabber::hasAlgae);
    }

    public Command L2(BooleanSupplier confirm) {
        return scoreCoral(Elevator.ElevatorState.L2, Arm.ArmState.SCORE, 0.5, confirm);
    }

    public Command L3(BooleanSupplier confirm) {
        return scoreCoral(Elevator.ElevatorState.L3, Arm.ArmState.SCORE, 0.5, confirm);
    }

    public Command L4(BooleanSupplier confirm) {
        return scoreCoral(Elevator.ElevatorState.L4, Arm.ArmState.L4_SCORE, 0.5, confirm);
    }

    public Command L2Reverse(BooleanSupplier confirm) {
        return scoreCoral(Elevator.ElevatorState.L2, Arm.ArmState.SCORE_REVERSE, -0.5, confirm);
    }

    public Command L3Reverse(BooleanSupplier confirm) {
        return scoreCoral(Elevator.ElevatorState.L3, Arm.ArmState.SCORE_REVERSE, -0.5, confirm);
    }

    public Command L4Reverse(BooleanSupplier confirm) {
        return scoreCoral(Elevator.ElevatorState.L4, Arm.ArmState.L4_SCORE_REVERSE, -0.5, confirm);
    }

    public Command L1(BooleanSupplier confirm) {
        return scoreCoral(Elevator.ElevatorState.L1, Arm.ArmState.L1_SCORE, -0.2, confirm);
    }

    public Command dealgifyLow() {
        return Commands.sequence(
                arm.commandToSetpoint(Arm.ArmState.STOW),
                elevator.commandToSetpoint(Elevator.ElevatorState.LOW_ALGAE),
                arm.commandToSetpoint(Arm.ArmState.REEF_ALGAE),
                Commands.runOnce(() -> grabber.setPercent(0.25)),
                Commands.waitUntil(grabber::hasAlgae),
                stow());
    }

    public Command dealgifyHigh() {
        return Commands.sequence(
                arm.commandToSetpoint(Arm.ArmState.STOW),
                elevator.commandToSetpoint(Elevator.ElevatorState.HIGH_ALGAE),
                arm.commandToSetpoint(Arm.ArmState.REEF_ALGAE),
                Commands.runOnce(() -> grabber.setPercent(0.25)),
                Commands.waitUntil(grabber::hasAlgae),
                stow());
    }

    public Command coralGroundIntake() {
        return Commands.sequence(
                        stow(),
                        coralIntake.commandToSetpoint(CoralIntake.CoralIntakeState.INTAKE),
                        arm.commandToSetpoint(Arm.ArmState.INTAKE),
                        grabber.commandRun(0.5).until(grabber::hasCoral),
                        stow())
                .unless(grabber::hasAlgae);
    }

    public Command sourceIntake() {
        return Commands.sequence(
                        arm.commandToSetpoint(Arm.ArmState.STOW),
                        elevator.commandToSetpoint(Elevator.ElevatorState.SOURCE),
                        arm.commandToSetpoint(Arm.ArmState.SOURCE_INTAKE),
                        grabber.commandRun(0.5).until(grabber::hasCoral),
                        stow())
                .unless(grabber::hasAlgae);
    }

    public Command algaeIntake() {
        return Commands.sequence(
                arm.commandToSetpoint(Arm.ArmState.STOW),
                elevator.commandToSetpoint(Elevator.ElevatorState.FLOOR_ALGAE),
                arm.commandToSetpoint(Arm.ArmState.FLOOR_ALGAE),
                Commands.runOnce(() -> grabber.setPercent(0.25)),
                Commands.waitUntil(grabber::hasAlgae),
                stow());
    }

    public Command processor(BooleanSupplier confirm) {
        return Commands.sequence(
                        stow(),
                        arm.commandToSetpoint(Arm.ArmState.PROCESSOR),
                        Commands.waitUntil(confirm),
                        grabber.commandRun(-0.5).withTimeout(0.5),
                        stow())
                .onlyIf(grabber::hasAlgae);
    }

    public Command net() {
        return Commands.sequence(
                arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE),
                elevator.commandToSetpoint(Elevator.ElevatorState.L3),
                arm.commandToSetpoint(Arm.ArmState.SCORE_REVERSE),
                Commands.parallel(
                        elevator.commandToSetpoint(Elevator.ElevatorState.BARGE),
                        arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE),
                        grabber.commandRun(-0.5).withTimeout(1)),
                stow());
    }

    public Command netReverse() {
        return Commands.sequence(
                arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE),
                Commands.parallel(
                        elevator.commandToSetpoint(Elevator.ElevatorState.BARGE),
                        Commands.parallel(
                                        arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE),
                                        grabber.commandRun(-0.5).withTimeout(1))
                                .beforeStarting(Commands.waitUntil(() ->
                                        elevator.getPosition() > Elevator.ElevatorState.L3.height.getAsDouble()))),
                stow());
    }
}

package org.team1540.robot2025.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.robot2025.FieldConstants;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.arm.Arm.ArmState;
import org.team1540.robot2025.subsystems.arm.ArmConstants;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.elevator.Elevator.ElevatorState;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.Intake;
import org.team1540.robot2025.subsystems.intake.Intake.IntakeState;
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.Controllers;

public class Superstructure {
    public enum SuperstructureState {
        STOW(ArmState.STOW, ElevatorState.STOW, IntakeState.STOW),
        STOW_ALGAE(ArmState.STOW_ALGAE, ElevatorState.STOW_ALGAE, IntakeState.STOW),
        INTAKE_GROUND(ArmState.INTAKE, ElevatorState.GROUND_CORAL, IntakeState.INTAKE),
        INTAKE_GROUND_L1(ArmState.INTAKE, ElevatorState.GROUND_CORAL_L1, IntakeState.INTAKE),
        INTAKE_SOURCE(ArmState.STOW, ElevatorState.STOW, IntakeState.STOW),
        INTAKE_ALGAE(ArmState.GROUND_ALGAE, ElevatorState.GROUND_ALGAE, IntakeState.STOW),
        CORAL_EJECT(ArmState.STOW, ElevatorState.STOW, IntakeState.EJECT),

        L1_FRONT(ArmState.STOW, ElevatorState.STOW, IntakeState.L1),
        L1_BACK(ArmState.SCORE_L1_BACK, ElevatorState.L1_BACK, IntakeState.STOW),

        L2_FRONT(ArmState.SCORE_L2_L3_FRONT, ElevatorState.L2_FRONT, IntakeState.STOW),
        L2_BACK(ArmState.SCORE_L2_L3_BACK, ElevatorState.L2_BACK, IntakeState.STOW),

        L2_L3_FRONT_STAGE(ArmState.SCORE_L2_L3_FRONT, ElevatorState.FRONT_STAGE, IntakeState.STOW),
        L2_L3_BACK_STAGE(ArmState.SCORE_L2_L3_BACK, ElevatorState.BACK_STAGE, IntakeState.STOW),

        L3_FRONT(ArmState.SCORE_L2_L3_FRONT, ElevatorState.L3_FRONT, IntakeState.STOW),
        L3_BACK(ArmState.SCORE_L2_L3_BACK, ElevatorState.L3_BACK, IntakeState.STOW),

        L4_FRONT(ArmState.SCORE_L4_FRONT, ElevatorState.L4_FRONT, IntakeState.STOW),
        L4_BACK(ArmState.SCORE_L4_BACK, ElevatorState.L4_BACK, IntakeState.STOW),

        L4_FRONT_STAGE(ArmState.SCORE_L4_FRONT, ElevatorState.FRONT_STAGE, IntakeState.STOW),
        L4_BACK_STAGE(ArmState.SCORE_L4_BACK, ElevatorState.BACK_STAGE, IntakeState.STOW),

        DEALGIFY_LOW_FRONT(ArmState.REEF_ALGAE_FRONT, ElevatorState.REEF_ALGAE_LOW_FRONT, IntakeState.STOW),
        DEALGIFY_LOW_BACK(ArmState.REEF_ALGAE_BACK, ElevatorState.REEF_ALGAE_LOW_BACK, IntakeState.STOW),

        DEALGIFY_HIGH_FRONT(ArmState.REEF_ALGAE_FRONT, ElevatorState.REEF_ALGAE_HIGH_FRONT, IntakeState.STOW),
        DEALGIFY_HIGH_BACK(ArmState.REEF_ALGAE_BACK, ElevatorState.REEF_ALGAE_HIGH_BACK, IntakeState.STOW),

        CLEAN_FRONT(ArmState.CLEAN_ALGAE_FRONT, ElevatorState.CLEAN_ALGAE_FRONT, IntakeState.STOW),
        CLEAN_BACK(ArmState.CLEAN_ALGAE_BACK, ElevatorState.CLEAN_ALGAE_BACK, IntakeState.STOW),

        CLEAN_FRONT_STAGE(ArmState.CLEAN_ALGAE_FRONT_STAGE, ElevatorState.CLEAN_ALGAE_FRONT_STAGE, IntakeState.STOW),
        CLEAN_BACK_STAGE(ArmState.CLEAN_ALGAE_BACK_STAGE, ElevatorState.CLEAN_ALGAE_BACK_STAGE, IntakeState.STOW),
        // barge is same from both sides
        SCORE_BARGE_FRONT(ArmState.SCORE_BARGE_FRONT, ElevatorState.BARGE, IntakeState.STOW),
        SCORE_BARGE_BACK(ArmState.SCORE_BARGE_BACK, ElevatorState.BARGE, IntakeState.STOW),

        // no processor front (?)
        PROCESSOR_BACK(ArmState.PROCESSOR, ElevatorState.PROCESSOR, IntakeState.STOW);

        public final ArmState armState;
        public final ElevatorState elevatorState;
        public final IntakeState intakeState;

        SuperstructureState(ArmState armState, ElevatorState elevatorState, IntakeState intakeState) {
            this.armState = armState;
            this.elevatorState = elevatorState;
            this.intakeState = intakeState;
        }
    }

    public final Elevator elevator;
    public final Arm arm;
    public final Intake intake;
    public final Grabber grabber;
    private final double clearanceHeight = 0.45;

    private SuperstructureState goalState = SuperstructureState.STOW;

    public Superstructure(Elevator elevator, Arm arm, Intake intake, Grabber grabber) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.grabber = grabber;
    }

    @AutoLogOutput(key = "Superstructure/GoalState")
    public SuperstructureState getGoalState() {
        return goalState;
    }

    private Translation2d getEndEffectorPosition(double elevatorHeight, Rotation2d armAngle) {
        return new Translation2d(0, elevatorHeight).plus(new Translation2d(ArmConstants.ARM_LENGTH_METERS, armAngle));
    }

    public Command commandToState(SuperstructureState goalState) {
        return Commands.defer(
                () -> {
                    Command command = Commands.none();
                    this.goalState = goalState;
                    final ArmState armState;
                    final ElevatorState elevatorState;
                    if (grabber.hasAlgae() && goalState.armState == ArmState.STOW) {
                        armState = ArmState.STOW_ALGAE;
                    } else armState = goalState.armState;
                    if (grabber.hasAlgae() && goalState.elevatorState == ElevatorState.STOW) {
                        elevatorState = ElevatorState.STOW_ALGAE;
                    } else elevatorState = goalState.elevatorState;

                    if (grabber.hasAlgae()
                            && !((armState.position().getDegrees()
                                                    >= ArmState.STOW_ALGAE
                                                            .position()
                                                            .getDegrees()
                                            && arm.getPosition().getDegrees()
                                                    >= ArmState.STOW.position().getDegrees())
                                    || (armState.position().getDegrees() <= 100
                                            && arm.getPosition().getDegrees() <= 100))
                            && (elevatorState.height.getAsDouble() < clearanceHeight)) {
                        command = command.andThen(
                                elevator.commandToSetpoint(ElevatorState.L2_FRONT), arm.commandToSetpoint(armState));
                    }
                    if (elevatorState.height.getAsDouble() >= clearanceHeight) {
                        if (armState.position().getDegrees()
                                >= ArmState.STOW.position().getDegrees()) {
                            command = command.andThen(Commands.parallel(
                                    elevator.commandToSetpoint(elevatorState),
                                    Commands.waitUntil(
                                                    () -> getEndEffectorPosition(
                                                                            elevator.getPosition(), armState.position())
                                                                    .getY()
                                                            >= 0.1
                                                    //                                                            &&
                                                    // elevator.timeToSetpoint()
                                                    //
                                                    //  <= arm.timeToSetpoint(armState.position())
                                                    )
                                            .andThen(arm.commandToSetpoint(armState)),
                                    intake.commandToSetpoint(goalState.intakeState)));
                        } else {
                            command = command.andThen(Commands.parallel(
                                    elevator.commandToSetpoint(elevatorState),
                                    Commands.waitUntil(
                                                    () -> getEndEffectorPosition(
                                                                            elevator.getPosition(), armState.position())
                                                                    .getY()
                                                            > clearanceHeight
                                                    //                                                            &&
                                                    // elevator.timeToSetpoint()
                                                    //
                                                    //  <= arm.timeToSetpoint(armState.position())
                                                    )
                                            .andThen(Commands.parallel(
                                                    intake.commandToSetpoint(goalState.intakeState),
                                                    arm.commandToSetpoint(armState)))));
                        }
                    } else if (armState.position().getDegrees()
                                    >= ArmState.STOW.position().getDegrees()
                            && armState.position().getDegrees() <= 150) {
                        command = command.andThen(Commands.parallel(
                                arm.commandToSetpoint(armState),
                                Commands.waitUntil(() -> (arm.getPosition().getDegrees()
                                                                >= ArmState.STOW
                                                                        .position()
                                                                        .getDegrees()
                                                        && arm.getPosition().getDegrees() <= 150)
                                                || arm.timeToSetpoint() + 0.1
                                                        <= elevator.timeToSetpoint(clearanceHeight))
                                        .andThen(elevator.commandToSetpoint(elevatorState)),
                                Commands.waitUntil(() -> (arm.getPosition().getDegrees()
                                                                >= ArmState.STOW
                                                                        .position()
                                                                        .getDegrees()
                                                        && arm.getPosition().getDegrees() <= 150)
                                                || arm.timeToSetpoint()
                                                        <= intake.timeToSetpoint(goalState.intakeState.pivotPosition()))
                                        .andThen(intake.commandToSetpoint(goalState.intakeState))));
                    } else if (goalState.intakeState.pivotPosition().getDegrees() < 80) {
                        command = command.andThen(Commands.parallel(
                                intake.commandToSetpoint(goalState.intakeState),
                                Commands.waitUntil(() -> intake.timeToSetpoint() + 0.1
                                                <= arm.timeToSetpoint(armState.position()))
                                        .andThen(Commands.parallel(
                                                elevator.commandToSetpoint(elevatorState),
                                                arm.commandToSetpoint(armState)))));
                    } else {
                        command = command.andThen(
                                commandStowArm(),
                                elevator.commandToSetpoint(elevatorState),
                                intake.commandToSetpoint(goalState.intakeState),
                                arm.commandToSetpoint(armState));
                    }

                    return command;
                },
                Set.of(arm, elevator, intake));
    }

    public Command commandStowArm() {
        return new ConditionalCommand(
                arm.commandToSetpoint(ArmState.STOW_ALGAE), arm.commandToSetpoint(ArmState.STOW), grabber::hasAlgae);
    }

    public Command stow() {
        return commandToState(SuperstructureState.STOW);
    }

    public Command preScoreCoral(FieldConstants.ReefHeight height, BooleanSupplier shouldReverse) {
        return switch (height) {
            case L1 -> Commands.none();
            case L2, L3 -> Commands.either(
                    commandToState(SuperstructureState.L2_L3_FRONT_STAGE),
                    commandToState(SuperstructureState.L2_L3_BACK_STAGE),
                    shouldReverse);
            case L4 -> Commands.either(
                    commandToState(SuperstructureState.L4_FRONT_STAGE),
                    commandToState(SuperstructureState.L4_BACK_STAGE),
                    shouldReverse);
        };
    }

    public Command scoreCoral(FieldConstants.ReefHeight height, BooleanSupplier shouldReverse) {
        return switch (height) {
            case L1 -> L1();
            case L2 -> L2(shouldReverse);
            case L3 -> L3(shouldReverse);
            case L4 -> L4(shouldReverse);
        };
    }

    public Command L1() {
        return commandToState(SuperstructureState.L1_FRONT);
    }

    public Command L2(BooleanSupplier shouldReverse) {
        return Commands.either(
                commandToState(SuperstructureState.L2_FRONT),
                commandToState(SuperstructureState.L2_BACK),
                shouldReverse);
    }

    public Command L3(BooleanSupplier shouldReverse) {
        return Commands.either(
                commandToState(SuperstructureState.L3_FRONT),
                commandToState(SuperstructureState.L3_BACK),
                shouldReverse);
    }

    public Command L4(BooleanSupplier shouldReverse) {
        return Commands.either(
                commandToState(SuperstructureState.L4_FRONT),
                commandToState(SuperstructureState.L4_BACK),
                shouldReverse);
    }

    public Command score() {
        return score(true);
    }

    public Command score(boolean stow) {
        return Commands.defer(
                        () -> switch (getGoalState()) {
                            case L1_FRONT -> intake.commandRunRollerFunnel(-0.2, -0.2)
                                    .withDeadline(Commands.waitUntil(() -> !intake.hasCoral())
                                            .andThen(Commands.waitSeconds(0.5)));
                            case L2_FRONT, L3_FRONT -> grabber.commandRun(-0.25)
                                    .withDeadline(Commands.waitUntil(() -> !grabber.forwardSensorTripped())
                                            .andThen(Commands.waitSeconds(0.05)));
                            case L4_FRONT -> grabber.commandRun(0.1)
                                    .until(grabber::reverseSensorTripped)
                                    .withTimeout(0.1)
                                    .onlyIf(() -> grabber.forwardSensorTripped() && !grabber.reverseSensorTripped())
                                    .andThen(grabber.commandRun(-0.3)
                                            .withDeadline(Commands.waitUntil(() -> !grabber.forwardSensorTripped())
                                                    .andThen(Commands.waitSeconds(0.25)))
                                            .alongWith(
                                                    Commands.waitSeconds(0.1),
                                                    arm.commandToSetpoint(ArmState.BACKOFF_L4_FRONT)));
                            case L4_BACK -> grabber.commandRun(-0.1)
                                    .until(grabber::forwardSensorTripped)
                                    .withTimeout(0.1)
                                    .onlyIf(() -> !grabber.forwardSensorTripped() && grabber.reverseSensorTripped())
                                    .andThen(grabber.commandRun(0.35)
                                            .withDeadline(Commands.waitUntil(() -> !grabber.reverseSensorTripped())
                                                    .andThen(Commands.waitSeconds(0.25)))
                                            .alongWith(
                                                    Commands.waitSeconds(0.2),
                                                    arm.commandToSetpoint(ArmState.BACKOFF_L4_BACK)));
                            case L1_BACK, L2_BACK, L3_BACK -> grabber.commandRun(0.6)
                                    .withDeadline(Commands.waitUntil(() -> !grabber.reverseSensorTripped())
                                            .andThen(Commands.waitSeconds(0.25)));
                            case PROCESSOR_BACK -> grabber.commandRun(-0.3).withTimeout(0.5);
                            case SCORE_BARGE_BACK -> grabber.commandRun(-1.0)
                                    .withTimeout(0.5)
                                    .alongWith(Commands.runOnce(arm::holdPosition));
                            case SCORE_BARGE_FRONT -> grabber.commandRun(-1.0)
                                    .withTimeout(0.5)
                                    .alongWith(Commands.waitSeconds(0.4)
                                            .andThen(arm.commandToSetpoint(ArmState.BACKOFF_BARGE_FRONT)));
                            default -> grabber.hasAlgae()
                                    ? grabber.commandRun(-0.5).withTimeout(0.5)
                                    : grabber.commandStartRun(0);
                        },
                        Set.of(elevator, arm, intake, grabber))
                .alongWith(Controllers.getInstance().rumbleDriver())
                .andThen(stow().onlyIf(() -> stow));
    }

    private Command dealgify(SuperstructureState state) {
        return Commands.defer(
                () -> {
                    if (state != SuperstructureState.DEALGIFY_HIGH_BACK
                            && state != SuperstructureState.DEALGIFY_HIGH_FRONT
                            && state != SuperstructureState.DEALGIFY_LOW_BACK
                            && state != SuperstructureState.DEALGIFY_LOW_FRONT) return Commands.none();
                    return Commands.parallel(commandToState(state), Commands.runOnce(() -> grabber.setPercent(0.5)))
                            .unless(grabber::reverseSensorTripped);
                },
                Set.of(elevator, arm, intake, grabber));
    }

    public Command dealgifyLow() {
        return dealgify(SuperstructureState.DEALGIFY_LOW_BACK);
    }

    public Command dealgifyHigh() {
        return dealgify(SuperstructureState.DEALGIFY_HIGH_BACK);
    }

    public Command dealgify(FieldConstants.ReefFace face) {
        return Commands.defer(
                () -> {
                    if (!face.highDealgify()) {
                        if (RobotState.getInstance().shouldReverseAlgae(face))
                            return dealgify(SuperstructureState.DEALGIFY_LOW_FRONT);
                        else return dealgify(SuperstructureState.DEALGIFY_LOW_BACK);
                    } else {
                        if (RobotState.getInstance().shouldReverseAlgae(face))
                            return dealgify(SuperstructureState.DEALGIFY_HIGH_FRONT);
                        else return dealgify(SuperstructureState.DEALGIFY_HIGH_BACK);
                    }
                },
                Set.of(elevator, arm, intake, grabber));
    }

    public Command clean(FieldConstants.ReefFace face) {
        return Commands.defer(
                () -> {
                    if (RobotState.getInstance().shouldReverseAlgae(face))
                        return commandToState(SuperstructureState.CLEAN_FRONT);
                    else return commandToState(SuperstructureState.CLEAN_BACK);
                },
                Set.of(elevator, arm, intake, grabber));
    }

    public Command cleanStage(FieldConstants.ReefFace face) {
        return Commands.defer(
                () -> {
                    if (RobotState.getInstance().shouldReverseAlgae(face))
                        return commandToState(SuperstructureState.CLEAN_FRONT_STAGE);
                    else return commandToState(SuperstructureState.CLEAN_BACK_STAGE);
                },
                Set.of(elevator, arm, intake, grabber));
    }

    public Command coralGroundIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_GROUND)
                                .withTimeout(1.0)
                                .deadlineFor(Commands.startEnd(
                                                () -> intake.setRollerVoltage(0.75 * 12),
                                                () -> intake.setRollerVoltage(0.0))
                                        .unless(intake::hasCoral)),
                        grabber.commandRun(0.3)
                                .until(grabber::forwardSensorTripped)
                                .andThen(grabber.commandRun(0.1).until(grabber::reverseSensorTripped))
                                .deadlineFor(intake.commandRunRollerFunnel(0.75, 0.75)),
                        stow().alongWith(
                                        grabber.commandRun(0.0),
                                        Commands.startEnd(
                                                        () -> {
                                                            intake.setRollerVoltage(-0.75 * 12);
                                                        },
                                                        () -> {
                                                            intake.setRollerVoltage(0);
                                                        })
                                                .withTimeout(0.5)
                                                .onlyIf(grabber::reverseSensorTripped)))
                .unless(grabber::hasAlgae);
    }

    public Command coralGroundIntakeL1() {
        return Commands.sequence(
                commandToState(SuperstructureState.INTAKE_GROUND_L1).withTimeout(1.0),
                intake.commandRunRoller(0.7).until(intake::hasCoral),
                intake.commandRunRoller(0.2).withTimeout(0.2),
                stow());
    }

    public Command coralIntakeEject() {
        return Commands.parallel(
                commandToState(SuperstructureState.CORAL_EJECT),
                Commands.waitSeconds(0.1).andThen(intake.commandRunRollerFunnel(-0.5, -0.5)));
    }

    public Command coralIntakeReverseHandoff() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_GROUND),
                        grabber.commandRun(-0.1)
                                .until(() -> !grabber.forwardSensorTripped())
                                .deadlineFor(intake.commandRunRollerFunnel(0.25, -0.25)),
                        stow())
                .unless(grabber::hasAlgae);
    }

    public Command sourceIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_SOURCE),
                        intake.commandRunRoller(0.5).until(intake::hasCoral),
                        stow())
                .unless(grabber::hasAlgae);
    }

    public Command algaeIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_ALGAE).withTimeout(1.0),
                        Commands.runOnce(() -> grabber.setPercent(1.0)),
                        Commands.waitUntil(grabber::hasAlgae),
                        Commands.runOnce(() -> grabber.setPercent(0.25)),
                        stow())
                .unless(grabber::reverseSensorTripped);
    }

    public Command processor() {
        return commandToState(SuperstructureState.PROCESSOR_BACK);
    }

    public Command net() {
        return Commands.either(
                commandToState(SuperstructureState.SCORE_BARGE_BACK),
                commandToState(SuperstructureState.SCORE_BARGE_FRONT),
                () -> Math.abs(RobotState.getInstance().getRobotRotation().getDegrees()
                                - AllianceFlipUtil.maybeReverseRotation(Rotation2d.kZero)
                                        .getDegrees())
                        < 90);
    }

    public Command zeroCommand() {
        return Commands.sequence(
                arm.commandToSetpoint(ArmState.STOW), Commands.parallel(elevator.zeroCommand(), intake.zeroCommand()));
    }
}

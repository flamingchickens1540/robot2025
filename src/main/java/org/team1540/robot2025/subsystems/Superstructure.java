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

public class Superstructure {
    public enum SuperstructureState {
        STOW(ArmState.STOW, ElevatorState.STOW, IntakeState.STOW),
        STOW_ALGAE(ArmState.STOW_ALGAE, ElevatorState.STOW_ALGAE, IntakeState.STOW),
        INTAKE_GROUND(ArmState.INTAKE, ElevatorState.GROUND_CORAL, IntakeState.INTAKE),
        INTAKE_FUNNEL(ArmState.FUNNEL, ElevatorState.FUNNEL, IntakeState.STOW),
        INTAKE_ALGAE(ArmState.GROUND_ALGAE, ElevatorState.GROUND_ALGAE, IntakeState.STOW),
        CORAL_EJECT(ArmState.STOW, ElevatorState.STOW, IntakeState.EJECT),

        L1_BACK(ArmState.SCORE_L1_BACK, ElevatorState.L1_BACK, IntakeState.STOW),
        L1_FRONT(ArmState.SCORE_L1_FRONT, ElevatorState.L1_FRONT, IntakeState.STOW),

        L2_FRONT(ArmState.SCORE_L2_L3_FRONT, ElevatorState.L2, IntakeState.STOW),
        L2_BACK(ArmState.SCORE_L2_L3_BACK, ElevatorState.L2, IntakeState.STOW),

        L3_FRONT(ArmState.SCORE_L2_L3_FRONT, ElevatorState.L3, IntakeState.STOW),
        L3_BACK(ArmState.SCORE_L2_L3_BACK, ElevatorState.L3, IntakeState.STOW),

        L4_FRONT(ArmState.SCORE_L4_FRONT, ElevatorState.L4, IntakeState.STOW),
        L4_BACK(ArmState.SCORE_L4_BACK, ElevatorState.L4, IntakeState.STOW),

        DEALGIFY_LOW_BACK(ArmState.REEF_ALGAE_BACK, ElevatorState.REEF_ALGAE_LOW, IntakeState.STOW),
        DEALGIFY_LOW_FRONT(ArmState.REEF_ALGAE_FRONT, ElevatorState.REEF_ALGAE_LOW, IntakeState.STOW),

        DEALGIFY_HIGH_FRONT(ArmState.REEF_ALGAE_FRONT, ElevatorState.REEF_ALGAE_HIGH, IntakeState.STOW),
        DEALGIFY_HIGH_BACK(ArmState.REEF_ALGAE_BACK, ElevatorState.REEF_ALGAE_HIGH, IntakeState.STOW),

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

    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;
    private final Grabber grabber;
    private final double clearanceHeight = 0.5;

    private SuperstructureState goalState;

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

    private Translation2d eeCoords(double elevatorHeight, Rotation2d armAngle) {
        Translation2d translation2d =
                new Translation2d(0, elevatorHeight).plus(new Translation2d(ArmConstants.ARM_LENGTH_METERS, armAngle));
        //        System.out.println("EEX: " + translation2d.getX() + " EEY: " + translation2d.getY());
        return translation2d;
    }

    public Command commandToState(SuperstructureState goalState) {
        return Commands.defer(
                () -> {
                    Command command = Commands.none();
                    this.goalState = goalState;

                    if (grabber.hasAlgae()
                            && !((goalState.armState.position().getDegrees()
                                                    >= ArmState.STOW_ALGAE
                                                            .position()
                                                            .getDegrees()
                                            && arm.getPosition().getDegrees()
                                                    >= ArmState.STOW_ALGAE
                                                            .position()
                                                            .getDegrees())
                                    || (goalState.armState.position().getDegrees() <= 100
                                            && arm.getPosition().getDegrees() <= 100))) {
                        command = command.andThen(
                                elevator.commandToSetpoint(ElevatorState.L2),
                                arm.commandToSetpoint(goalState.armState));
                    }
                    if (goalState.elevatorState.height.getAsDouble() >= clearanceHeight) {
                        if (goalState.armState.position().getDegrees()
                                >= ArmState.STOW.position().getDegrees()) {
                            command = command.andThen(Commands.parallel(
                                    elevator.commandToSetpoint(goalState.elevatorState),
                                    Commands.waitUntil(() ->
                                                    eeCoords(elevator.getPosition(), goalState.armState.position())
                                                                            .getY()
                                                                    >= 0.1
                                                            && elevator.timeToSetpoint()
                                                                    <= arm.timeToSetpoint(
                                                                            goalState.armState.position()))
                                            .andThen(arm.commandToSetpoint(goalState.armState)),
                                    intake.commandToSetpoint(goalState.intakeState)));
                        } else {
                            command = command.andThen(Commands.parallel(
                                    elevator.commandToSetpoint(goalState.elevatorState),
                                    Commands.waitUntil(() ->
                                                    eeCoords(elevator.getPosition(), goalState.armState.position())
                                                                            .getY()
                                                                    > clearanceHeight
                                                            && elevator.timeToSetpoint()
                                                                    <= arm.timeToSetpoint(
                                                                            goalState.armState.position()))
                                            .andThen(Commands.parallel(
                                                    intake.commandToSetpoint(goalState.intakeState),
                                                    arm.commandToSetpoint(goalState.armState)))));
                        }
                    } else if (goalState.armState.position().getDegrees()
                                    >= ArmState.STOW.position().getDegrees()
                            && goalState.armState.position().getDegrees() <= 150) {
                        command = command.andThen(Commands.parallel(
                                arm.commandToSetpoint(goalState.armState),
                                Commands.waitUntil(() -> (arm.getPosition().getDegrees()
                                                                >= ArmState.STOW
                                                                        .position()
                                                                        .getDegrees()
                                                        && arm.getPosition().getDegrees() <= 150)
                                                || arm.timeToSetpoint() + 0.1
                                                        <= elevator.timeToSetpoint(clearanceHeight))
                                        .andThen(Commands.parallel(
                                                intake.commandToSetpoint(goalState.intakeState),
                                                elevator.commandToSetpoint(goalState.elevatorState)))));
                    } else if (goalState.intakeState.pivotPosition().getDegrees() < 80) {
                        command = command.andThen(Commands.parallel(
                                intake.commandToSetpoint(goalState.intakeState),
                                Commands.waitUntil(() -> intake.timeToSetpoint() + 0.1
                                                <= arm.timeToSetpoint(goalState.armState.position()))
                                        .andThen(Commands.parallel(
                                                elevator.commandToSetpoint(goalState.elevatorState),
                                                arm.commandToSetpoint(goalState.armState)))));
                    } else {
                        command = command.andThen(
                                Commands.print("Superstructure Else"),
                                commandStowArm(),
                                elevator.commandToSetpoint(goalState.elevatorState),
                                intake.commandToSetpoint(goalState.intakeState),
                                arm.commandToSetpoint(goalState.armState));
                    }

                    return command;
                    //                    }
                },
                Set.of(arm, elevator, intake));
    }
    // TODO: Simon's fun thing where you can start moving arm early

    @AutoLogOutput(key = "Superstructure/ArmClear")
    public boolean isArmClear() {
        return elevator.getPosition() > clearanceHeight;
    }

    public Command commandStowArm() {
        return new ConditionalCommand(
                arm.commandToSetpoint(ArmState.STOW_ALGAE), arm.commandToSetpoint(ArmState.STOW), grabber::hasAlgae);
    }

    private Command scoreCoral(SuperstructureState superstructureState, double grabberPower, BooleanSupplier confirm) {
        return Commands.sequence(
                        commandToState(superstructureState),
                        Commands.waitUntil(confirm),
                        grabber.commandRun(grabberPower).until(() -> !grabber.reverseSensorTripped()),
                        grabber.commandRun(grabberPower).withTimeout(0.1),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::hasAlgae);
    }

    public Command scoreCoral(FieldConstants.ReefHeight height, BooleanSupplier confirm) {
        return switch (height) {
            case L1 -> L1(confirm);
            case L2 -> L2(confirm);
            case L3 -> L3(confirm);
            case L4 -> L4(confirm);
        };
    }

    public Command L1(BooleanSupplier confirm) {
        return Commands.defer(
                () -> {
                    if (Math.abs(FieldConstants.Reef.closestFace()
                                    .get()
                                    .getRotation()
                                    .minus(RobotState.getInstance().getRobotRotation())
                                    .getDegrees())
                            < 90) return scoreCoral(SuperstructureState.L1_BACK, 0.3, confirm);
                    else return scoreCoral(SuperstructureState.L1_FRONT, -0.3, confirm);
                },
                Set.of(elevator, arm, intake, grabber));
    }

    public Command L2(BooleanSupplier confirm) {
        return Commands.defer(
                () -> {
                    if (Math.abs(FieldConstants.Reef.closestFace()
                                    .get()
                                    .getRotation()
                                    .minus(RobotState.getInstance().getRobotRotation())
                                    .getDegrees())
                            < 90) return scoreCoral(SuperstructureState.L2_BACK, 0.3, confirm);
                    else return scoreCoral(SuperstructureState.L2_FRONT, -0.3, confirm);
                },
                Set.of(elevator, arm, intake, grabber));
    }

    public Command L3(BooleanSupplier confirm) {
        return Commands.defer(
                () -> {
                    if (Math.abs(FieldConstants.Reef.closestFace()
                                    .get()
                                    .getRotation()
                                    .minus(RobotState.getInstance().getRobotRotation())
                                    .getDegrees())
                            < 90) return scoreCoral(SuperstructureState.L3_BACK, 0.3, confirm);
                    else return scoreCoral(SuperstructureState.L3_FRONT, -0.3, confirm);
                },
                Set.of(elevator, arm, intake, grabber));
    }

    public Command L4(BooleanSupplier confirm) {
        return Commands.defer(
                () -> {
                    if (Math.abs(FieldConstants.Reef.closestFace()
                                    .get()
                                    .getRotation()
                                    .minus(RobotState.getInstance().getRobotRotation())
                                    .getDegrees())
                            < 90) return scoreCoral(SuperstructureState.L4_BACK, 0.3, confirm);
                    else return scoreCoral(SuperstructureState.L4_FRONT, -0.3, confirm);
                },
                Set.of(elevator, arm, intake, grabber));
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

    //    public Command L1(BooleanSupplier confirm) {
    //        return scoreCoral(SuperstructureState.L1_BACK, -0.2, confirm);
    //    }

    private Command dealgify(SuperstructureState state) {
        return Commands.defer(
                () -> {
                    if (state != SuperstructureState.DEALGIFY_HIGH_BACK
                            && state != SuperstructureState.DEALGIFY_HIGH_FRONT
                            && state != SuperstructureState.DEALGIFY_LOW_BACK
                            && state != SuperstructureState.DEALGIFY_LOW_FRONT) return Commands.none();
                    return Commands.sequence(
                                    commandToState(state),
                                    Commands.runOnce(() -> grabber.setPercent(0.25)),
                                    Commands.waitUntil(grabber::hasAlgae))
                            .unless(grabber::reverseSensorTripped)
                            .handleInterrupt(grabber::stop);
                },
                Set.of(elevator, arm, intake, grabber));
    }

    public Command dealgifyLow() {
        return dealgify(SuperstructureState.DEALGIFY_LOW_BACK);
    }

    public Command dealgifyHigh() {
        return Commands.sequence(
                        commandToState(SuperstructureState.DEALGIFY_HIGH_BACK),
                        Commands.runOnce(() -> grabber.setPercent(0.25)),
                        Commands.waitUntil(grabber::hasAlgae))
                //                        commandToState(SuperstructureState.STOW))
                .unless(grabber::reverseSensorTripped)
                .handleInterrupt(grabber::stop);
    }

    public Command dealgifyHighFront() {
        return Commands.sequence(
                        commandToState(SuperstructureState.DEALGIFY_HIGH_FRONT),
                        Commands.runOnce(() -> grabber.setPercent(0.25)),
                        Commands.waitUntil(grabber::hasAlgae))
                //                        commandToState(SuperstructureState.STOW))
                .unless(grabber::reverseSensorTripped)
                .handleInterrupt(grabber::stop);
    }

    public Command dealgify() {
        return Commands.defer(
                () -> {
                    int degrees = (int) Math.round(FieldConstants.Reef.closestFace()
                            .get()
                            .getRotation()
                            .getDegrees());
                    if (degrees == -180 || degrees == 60 || degrees == -60) {
                        if (Math.abs(degrees
                                        - RobotState.getInstance()
                                                .getRobotRotation()
                                                .getDegrees())
                                < 90) return dealgify(SuperstructureState.DEALGIFY_LOW_BACK);
                        return dealgify(SuperstructureState.DEALGIFY_LOW_FRONT);
                    } else {
                        if (Math.abs(degrees
                                        - RobotState.getInstance()
                                                .getRobotRotation()
                                                .getDegrees())
                                < 90) return dealgify(SuperstructureState.DEALGIFY_HIGH_BACK);
                        else return dealgify(SuperstructureState.DEALGIFY_HIGH_FRONT);
                    }
                },
                Set.of(elevator, arm, intake, grabber));
    }

    public Command coralGroundIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_GROUND),
                        grabber.commandRun(0.3)
                                .until(grabber::forwardSensorTripped)
                                .andThen(grabber.commandRun(0.1).until(grabber::reverseSensorTripped))
                                .deadlineFor(intake.commandRunRollerFunnel(0.5, 0.5)),
                        commandToState(SuperstructureState.STOW).alongWith(grabber.commandRun(0.0)))
                .unless(grabber::hasAlgae);
    }

    public Command coralIntakeEject() {
        return Commands.sequence(
                        commandToState(SuperstructureState.CORAL_EJECT), intake.commandRunRollerFunnel(-0.5, -0.5))
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
                arm.commandToSetpoint(ArmState.STOW), Commands.parallel(elevator.zeroCommand(), intake.zeroCommand()));
    }
}

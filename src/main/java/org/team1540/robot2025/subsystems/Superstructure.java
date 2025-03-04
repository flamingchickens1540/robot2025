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
        INTAKE_GROUND_L1(ArmState.STOW, ElevatorState.STOW, IntakeState.INTAKE),
        INTAKE_SOURCE(ArmState.STOW, ElevatorState.STOW, IntakeState.STOW),
        INTAKE_ALGAE(ArmState.GROUND_ALGAE, ElevatorState.GROUND_ALGAE, IntakeState.STOW),
        CORAL_EJECT(ArmState.STOW, ElevatorState.STOW, IntakeState.EJECT),

        L1_FRONT(ArmState.STOW, ElevatorState.STOW, IntakeState.L1),
        L1_BACK(ArmState.SCORE_L1_BACK, ElevatorState.L1_BACK, IntakeState.STOW),

        L2_FRONT(ArmState.SCORE_L2_L3_FRONT, ElevatorState.L2_FRONT, IntakeState.STOW),
        L2_BACK(ArmState.SCORE_L2_L3_BACK, ElevatorState.L2_BACK, IntakeState.STOW),

        L3_FRONT(ArmState.SCORE_L2_L3_FRONT, ElevatorState.L3_FRONT, IntakeState.STOW),
        L3_BACK(ArmState.SCORE_L2_L3_BACK, ElevatorState.L3_BACK, IntakeState.STOW),

        L4_FRONT(ArmState.SCORE_L4_FRONT, ElevatorState.L4_FRONT, IntakeState.STOW),
        L4_BACK(ArmState.SCORE_L4_BACK, ElevatorState.L4_BACK, IntakeState.STOW),

        DEALGIFY_LOW_FRONT(ArmState.REEF_ALGAE_FRONT, ElevatorState.REEF_ALGAE_LOW_FRONT, IntakeState.STOW),
        DEALGIFY_LOW_BACK(ArmState.REEF_ALGAE_BACK, ElevatorState.REEF_ALGAE_LOW_BACK, IntakeState.STOW),

        DEALGIFY_HIGH_FRONT(ArmState.REEF_ALGAE_FRONT, ElevatorState.REEF_ALGAE_HIGH_FRONT, IntakeState.STOW),
        DEALGIFY_HIGH_BACK(ArmState.REEF_ALGAE_BACK, ElevatorState.REEF_ALGAE_HIGH_BACK, IntakeState.STOW),

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

    private Translation2d getEndEffectorPosition(double elevatorHeight, Rotation2d armAngle) {
        return new Translation2d(0, elevatorHeight).plus(new Translation2d(ArmConstants.ARM_LENGTH_METERS, armAngle));
    }

    public Command commandToState(SuperstructureState goalState) {
        return Commands.defer(
                () -> {
                    Command command = Commands.none();
                    this.goalState = goalState;
                    final ArmState armState;
                    if (grabber.hasAlgae() && goalState.armState == ArmState.STOW) {
                        armState = ArmState.STOW_ALGAE;
                    } else armState = goalState.armState;

                    if (grabber.hasAlgae()
                            && !((armState.position().getDegrees()
                                                    >= ArmState.STOW_ALGAE
                                                            .position()
                                                            .getDegrees()
                                            && arm.getPosition().getDegrees()
                                                    >= ArmState.STOW.position().getDegrees())
                                    || (armState.position().getDegrees() <= 100
                                            && arm.getPosition().getDegrees() <= 100))) {
                        command = command.andThen(
                                elevator.commandToSetpoint(ElevatorState.L2_FRONT), arm.commandToSetpoint(armState));
                    }
                    if (goalState.elevatorState.height.getAsDouble() >= clearanceHeight) {
                        if (armState.position().getDegrees()
                                >= ArmState.STOW.position().getDegrees()) {
                            command = command.andThen(Commands.parallel(
                                    elevator.commandToSetpoint(goalState.elevatorState),
                                    Commands.waitUntil(() ->
                                                    getEndEffectorPosition(elevator.getPosition(), armState.position())
                                                                            .getY()
                                                                    >= 0.1
                                                            && elevator.timeToSetpoint()
                                                                    <= arm.timeToSetpoint(armState.position()))
                                            .andThen(arm.commandToSetpoint(armState)),
                                    intake.commandToSetpoint(goalState.intakeState)));
                        } else {
                            command = command.andThen(Commands.parallel(
                                    elevator.commandToSetpoint(goalState.elevatorState),
                                    Commands.waitUntil(() ->
                                                    getEndEffectorPosition(elevator.getPosition(), armState.position())
                                                                            .getY()
                                                                    > clearanceHeight
                                                            && elevator.timeToSetpoint()
                                                                    <= arm.timeToSetpoint(armState.position()))
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
                                        .andThen(Commands.parallel(
                                                intake.commandToSetpoint(goalState.intakeState),
                                                elevator.commandToSetpoint(goalState.elevatorState)))));
                    } else if (goalState.intakeState.pivotPosition().getDegrees() < 80) {
                        command = command.andThen(Commands.parallel(
                                intake.commandToSetpoint(goalState.intakeState),
                                Commands.waitUntil(() -> intake.timeToSetpoint() + 0.1
                                                <= arm.timeToSetpoint(armState.position()))
                                        .andThen(Commands.parallel(
                                                elevator.commandToSetpoint(goalState.elevatorState),
                                                arm.commandToSetpoint(armState)))));
                    } else {
                        command = command.andThen(
                                commandStowArm(),
                                elevator.commandToSetpoint(goalState.elevatorState),
                                intake.commandToSetpoint(goalState.intakeState),
                                arm.commandToSetpoint(armState));
                    }

                    return command;
                },
                Set.of(arm, elevator, intake));
    }

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
                        grabber.commandRun(grabberPower).withTimeout(0.25),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::hasAlgae);
    }

    public Command scoreCoral(
            FieldConstants.ReefHeight height, BooleanSupplier confirm, BooleanSupplier shouldReverse) {
        return switch (height) {
            case L1 -> L1(confirm);
            case L2 -> L2(confirm, shouldReverse);
            case L3 -> L3(confirm, shouldReverse);
            case L4 -> L4(confirm, shouldReverse);
        };
    }

    public Command L1(BooleanSupplier confirm) {
        return Commands.sequence(
                commandToState(SuperstructureState.L1_FRONT),
                Commands.waitUntil(confirm),
                intake.commandRunRollerFunnel(-0.2, -0.2),
                commandToState(SuperstructureState.STOW));
    }

    public Command L2(BooleanSupplier confirm, BooleanSupplier shouldReverse) {
        return Commands.either(
                scoreCoral(SuperstructureState.L2_FRONT, -0.3, confirm),
                scoreCoral(SuperstructureState.L2_BACK, 0.3, confirm),
                shouldReverse);
    }

    public Command L3(BooleanSupplier confirm, BooleanSupplier shouldReverse) {
        return Commands.either(
                scoreCoral(SuperstructureState.L3_FRONT, -0.3, confirm),
                scoreCoral(SuperstructureState.L3_BACK, 0.3, confirm),
                shouldReverse);
    }

    public Command L4(BooleanSupplier confirm, BooleanSupplier shouldReverse) {
        return Commands.either(
                scoreCoral(SuperstructureState.L4_FRONT, -0.3, confirm),
                scoreCoral(SuperstructureState.L4_BACK, 0.3, confirm),
                shouldReverse);
    }

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
                                .deadlineFor(intake.commandRunRollerFunnel(0.8, 0.8)),
                        commandToState(SuperstructureState.STOW).alongWith(grabber.commandRun(0.0)))
                .unless(grabber::hasAlgae);
    }

    public Command coralGroundIntakeL1() {
        return Commands.sequence(
                commandToState(SuperstructureState.INTAKE_GROUND_L1),
                intake.commandRunRollerFunnel(0.8, 0.5).until(intake::hasCoral),
                commandToState(SuperstructureState.STOW));
    }

    public Command coralIntakeEject() {
        return Commands.sequence(
                        commandToState(SuperstructureState.CORAL_EJECT), intake.commandRunRollerFunnel(-0.5, -0.5))
                .unless(grabber::hasAlgae);
    }

    public Command sourceIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_SOURCE),
                        intake.commandRunRoller(0.5).until(intake::hasCoral),
                        commandToState(SuperstructureState.STOW))
                .unless(grabber::hasAlgae);
    }

    public Command algaeIntake() {
        return Commands.sequence(
                        commandToState(SuperstructureState.INTAKE_ALGAE),
                        Commands.runOnce(() -> grabber.setPercent(1.0)),
                        Commands.waitUntil(grabber::hasAlgae),
                        Commands.runOnce(() -> grabber.setPercent(0.25)),
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

    public Command net(BooleanSupplier confirm) {
        return Commands.sequence(
                commandToState(SuperstructureState.SCORE_BARGE_BACK),
                Commands.waitUntil(confirm),
                grabber.commandRun(-0.8),
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
                                        () -> elevator.getPosition() > ElevatorState.L3_BACK.height.getAsDouble()))),
                commandToState(SuperstructureState.STOW));
        //                .onlyIf(grabber::hasAlgae);
    }

    public Command zeroCommand() {
        return Commands.sequence(
                arm.commandToSetpoint(ArmState.STOW), Commands.parallel(elevator.zeroCommand(), intake.zeroCommand()));
    }
}

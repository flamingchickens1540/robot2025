package org.team1540.robot2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.robot2025.FieldConstants.ReefBranch;
import org.team1540.robot2025.FieldConstants.ReefHeight;
import org.team1540.robot2025.autos.Autos;
import org.team1540.robot2025.commands.AutoAlignCommands;
import org.team1540.robot2025.commands.AutoScoreCommands;
import org.team1540.robot2025.services.AlertManager;
import org.team1540.robot2025.services.MechanismVisualizer;
import org.team1540.robot2025.subsystems.Superstructure;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.climber.Climber;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.Intake;
import org.team1540.robot2025.subsystems.leds.CustomLEDPatterns;
import org.team1540.robot2025.subsystems.leds.Leds;
import org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVision;
import org.team1540.robot2025.subsystems.vision.coral.CoralVision;
import org.team1540.robot2025.util.*;
import org.team1540.robot2025.util.auto.LoggedAutoChooser;

public class RobotContainer {
    private final CommandXboxController driver = Controllers.getInstance().getDriver();
    private final CommandXboxController copilot = Controllers.getInstance().getCopilot();
    private final ButtonBoard buttonBoard = Controllers.getInstance().getButtonBoard();

    private final Drivetrain drivetrain;
    private final AprilTagVision aprilTagVision;
    private final CoralVision coralVision;
    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;
    private final Grabber grabber;
    private final Climber climber;
    private final Leds leds = new Leds();

    private final Superstructure superstructure;

    private final Autos autos;
    private final LoggedAutoChooser autoChooser = new LoggedAutoChooser("Auto Chooser");

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain = Drivetrain.createReal();
                aprilTagVision = AprilTagVision.createReal();
                coralVision = CoralVision.createReal();
                elevator = Elevator.createReal();
                arm = Arm.createReal();
                intake = Intake.createReal();
                grabber = Grabber.createReal();
                climber = Climber.createReal();
                break;
            case SIM:
                // Simulation, instantiate physics sim IO implementations
                drivetrain = Drivetrain.createSim();
                aprilTagVision = AprilTagVision.createSim();
                coralVision = CoralVision.createDummy();
                elevator = Elevator.createSim();
                arm = Arm.createSim();
                intake = Intake.createSim();
                grabber = Grabber.createSim();
                climber = Climber.createDummy();

                RobotState.getInstance().resetPose(new Pose2d(3.0, 3.0, Rotation2d.kZero));
                break;
            default:
                // Replayed robot, disable IO implementations
                drivetrain = Drivetrain.createDummy();
                aprilTagVision = AprilTagVision.createDummy();
                coralVision = CoralVision.createDummy();
                elevator = Elevator.createDummy();
                arm = Arm.createDummy();
                intake = Intake.createDummy();
                grabber = Grabber.createDummy();
                climber = Climber.createDummy();
        }
        superstructure = new Superstructure(elevator, arm, intake, grabber);
        autos = new Autos(drivetrain, superstructure);

        configureButtonBindings();
        configureAutoRoutines();
        configureRobotModeTriggers();
        configurePeriodicCallbacks();
        configureLEDBindings();
    }

    private void configureButtonBindings() {
        // Sim testing binding
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            driver.y()
                    .whileTrue(AutoScoreCommands.alignToBranchAndScore(
                            ReefBranch.E, ReefHeight.L2, drivetrain, superstructure));
            //            driver.b().whileTrue(AutoScoreCommands.alignToFaceAndClean(ReefBranch.E.face, drivetrain,
            // superstructure));
            //            driver.a()
            //                    .whileTrue(AutoScoreCommands.alignToFaceAndDealgify(ReefBranch.E.face, drivetrain,
            // superstructure));
            //            driver.b().whileTrue(AutoScoreCommands.alignToBargeAndScore(drivetrain, superstructure));
            //            driver.b().whileTrue(AutoScoreCommands.pointToBargeAndScore(drivetrain, superstructure,
            // driver.getHID()));

            //            RobotState.getInstance()
            //                    .addCoralObservation(new CoralVisionIO.CoralObservation(0, Rotation2d.kZero,
            // Rotation2d.kZero, 0));
            //            driver.b().whileTrue(drivetrain.seekAndDestroy());
        }
        //        driver.b().whileTrue(drivetrain.seekAndDestroy());
        driver.b().onTrue(Commands.runOnce(() -> RobotState.getInstance().toggleIntakeAssist()));

        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID(), () -> true));
        driver.x()
                .toggleOnTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(),
                        () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kCCW_90deg),
                        () -> true));
        driver.a().onTrue(superstructure.coralIntakeEject().withTimeout(2.0));
        driver.back().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        driver.leftStick().onTrue(superstructure.stow());

        driver.leftTrigger()
                .whileTrue(drivetrain
                        .teleopDriveIntakeAssistCommand(driver.getHID(), () -> true)
                        .onlyIf(RobotState.getInstance()::getIntakeAssist));

        driver.leftTrigger().whileTrue(Controllers.getInstance().rumbleDriver());
        new Trigger(intake::hasCoral)
                .onTrue(Controllers.getInstance().setDriverRumble(GenericHID.RumbleType.kBothRumble, 0));

        driver.leftTrigger()
                .and(buttonBoard.branchHeightAt(ReefHeight.L1).negate())
                .and(() -> !grabber.hasAlgae())
                .whileTrue(superstructure.coralGroundIntake())
                .onFalse(superstructure.stow());

        driver.leftTrigger()
                .and(buttonBoard.branchHeightAt(ReefHeight.L1).or(grabber::hasAlgae))
                .and(() -> !grabber.hasAlgae())
                .whileTrue(superstructure.coralGroundIntakeL1())
                .onFalse(superstructure.stow());

        driver.leftBumper()
                .whileTrue(superstructure.algaeIntake())
                .onFalse(superstructure.stow().unless(drivetrain::isAutoAligning));

        driver.rightTrigger().onTrue(superstructure.score());

        climber.setDefaultCommand(climber.climbCommand(() -> JoystickUtil.smartDeadzone(copilot.getRightY(), 0.1), 0));

        copilot.start()
                .whileTrue(superstructure
                        .zeroCommand()
                        .alongWith(Commands.runOnce(() -> climber.resetPosition(Rotation2d.kZero))));
        copilot.back()
                .toggleOnTrue(elevator.manualCommand(() -> 0.5 * -JoystickUtil.smartDeadzone(copilot.getLeftY(), 0.1)));
        copilot.rightTrigger().whileTrue(superstructure.coralGroundIntake()).onFalse(superstructure.stow());
        copilot.leftTrigger().onTrue(superstructure.stow());
        copilot.leftBumper().onTrue(superstructure.dealgifyHigh());
        copilot.rightBumper().onTrue(superstructure.dealgifyLow());

        copilot.y().onTrue(superstructure.L4(() -> true));
        copilot.x().onTrue(superstructure.L3(() -> true));
        copilot.a().onTrue(superstructure.L2(() -> true));
        copilot.povRight().onTrue(superstructure.L1());

        buttonBoard
                .button(1)
                .or(copilot.b())
                //                .or(driver.x())
                .onTrue(AutoScoreCommands.pointToBargeAndScore(drivetrain, superstructure, driver.getHID()));
        buttonBoard.button(2).onTrue(superstructure.processor());

        buttonBoard
                .button(3)
                //                .or(driver.x())
                .onTrue(AutoAlignCommands.alignToCage(FieldConstants.Barge.leftCage, drivetrain)
                        .andThen(drivetrain.teleopDriveWithHeadingCommand(
                                driver.getHID(),
                                () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kCCW_90deg),
                                () -> true))
                        .alongWith(superstructure.processor()));
        buttonBoard
                .button(4)
                .onTrue(AutoAlignCommands.alignToCage(FieldConstants.Barge.middleCage, drivetrain)
                        .andThen(drivetrain.teleopDriveWithHeadingCommand(
                                driver.getHID(),
                                () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kCCW_90deg),
                                () -> true))
                        .alongWith(superstructure.processor()));
        buttonBoard
                .button(5)
                .onTrue(AutoAlignCommands.alignToCage(FieldConstants.Barge.rightCage, drivetrain)
                        .andThen(drivetrain.teleopDriveWithHeadingCommand(
                                driver.getHID(),
                                () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kCCW_90deg),
                                () -> true))
                        .alongWith(superstructure.processor()));

        buttonBoard
                .button(6)
                .or(copilot.povRight())
                .toggleOnTrue(climber.climbCommand(() -> JoystickUtil.smartDeadzone(copilot.getRightY(), 0.1), 0.3)
                        .alongWith(superstructure.commandToState(Superstructure.SuperstructureState.PROCESSOR_BACK)));
        copilot.povDown().whileTrue(superstructure.coralIntakeEject()).onFalse(superstructure.stow());

        for (ButtonBoard.ReefButton button : ButtonBoard.ReefButton.values()) {
            for (ReefHeight height : ReefHeight.values()) {
                buttonBoard
                        .branchFaceAt(button)
                        .and(buttonBoard.branchHeightAt(height))
                        .and(driver.rightStick())
                        .whileTrue(AutoScoreCommands.alignToBranchAndScore(
                                buttonBoard.reefButtonToBranch(button), height, drivetrain, superstructure));
            }
            buttonBoard
                    .branchFaceAt(button)
                    .and(driver.rightBumper())
                    .and(buttonBoard.quickDealgify().negate())
                    .and(() -> !grabber.hasCoral())
                    .whileTrue(AutoScoreCommands.alignToFaceAndDealgify(
                            buttonBoard.reefButtonToBranch(button).face, drivetrain, superstructure));
            buttonBoard
                    .branchFaceAt(button)
                    .and(driver.rightBumper())
                    .and(buttonBoard.quickDealgify().or(grabber::hasCoral))
                    .whileTrue(AutoScoreCommands.alignToFaceAndClean(
                            buttonBoard.reefButtonToBranch(button).face, drivetrain, superstructure));
        }

        //        new Trigger(() -> RobotState.getInstance()
        //                                .getEstimatedPose()
        //                                .getTranslation()
        //
        // .getDistance(AllianceFlipUtil.maybeFlipTranslation(FieldConstants.Reef.center))
        //                        > FieldConstants.Reef.centerToZoneLine
        //                                + Units.inchesToMeters(12)
        //                                + Constants.BUMPER_LENGTH_X_METERS / 2)
        //                .and(() -> !(grabber.reverseSensorTripped() || grabber.forwardSensorTripped()))
        //                .and(() -> superstructure.getGoalState().elevatorState.height.getAsDouble() > 0.2)
        //                .onTrue(superstructure.stow());
    }

    private void configureAutoRoutines() {
        autoChooser.addCmd("Zero mechanisms", superstructure::zeroCommand);
        //        autoChooser.addRoutine("Right 3 Piece Lollipop", autos::right3PieceLollipop);
        //        autoChooser.addRoutine("Left 3 Piece Lollipop", autos::left3PieceLollipop);
        autoChooser.addRoutine("Right 3 Piece Sweep", autos::right3PieceSweep);
        autoChooser.addRoutine("Right 4 Piece Sweep Reverse", autos::right4PieceSweepReverse);
        autoChooser.addRoutine("Left 3 Piece Sweep", autos::left3PieceSweep);
        autoChooser.addRoutine("Left 4 Piece Sweep Reverse", autos::left4PieceSweepReverse);
        autoChooser.addRoutine("Left 4 Piece", autos::left4Piece);
        autoChooser.addCmd("Left 4 Piece Defer", autos::left4PieceSplit);
        autoChooser.addRoutine("Left 4 Piece Eyes", autos::left4PieceEyes);
        autoChooser.addRoutine("Center 1 Piece", autos::center1Piece);
        if (Constants.isTuningMode()) {
            autoChooser.addCmd("Drive FF Characterization", drivetrain::feedforwardCharacterization);
            autoChooser.addCmd("Drive Wheel Radius Characterization", drivetrain::wheelRadiusCharacterization);
            autoChooser.addCmd("Elevator FF Characterization", elevator::feedforwardCharacterizationCommand);
        }
    }

    private void configureRobotModeTriggers() {
        RobotModeTriggers.teleop()
                .and(DriverStation::isFMSAttached)
                .onTrue(Commands.runOnce(drivetrain::zeroFieldOrientation));
        RobotModeTriggers.teleop()
                .and(DriverStation::isFMSAttached)
                .onTrue(Commands.runOnce(() -> climber.resetPosition(Rotation2d.kZero)));
    }

    private void configurePeriodicCallbacks() {
        addPeriodicCallback(AlertManager.getInstance()::update, "AlertManager update");
        addPeriodicCallback(MechanismVisualizer.getInstance()::update, "MechanismVisualizer update");
        addPeriodicCallback(RobotState.getInstance()::periodicLog, "RobotState periodic log");
        addPeriodicCallback(autoChooser::update, "AutoChooser update");
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            addPeriodicCallback(SimState.getInstance()::update, "Simulation update");
        }
    }

    private void addPeriodicCallback(Runnable callback, String name) {
        CommandScheduler.getInstance()
                .schedule(Commands.run(callback).withName(name).ignoringDisable(true));
    }

    private void configureLEDBindings() {
        RobotModeTriggers.disabled()
                .onTrue(leds.viewFull.commandDefaultPattern(() -> CustomLEDPatterns.movingRainbow(Hertz.of(0.2))));
        RobotModeTriggers.autonomous()
                .onTrue(leds.viewFull.commandDefaultPattern(() -> LEDPattern.solid(Leds.getAllianceColor())));
        RobotModeTriggers.teleop()
                .onTrue(leds.viewFull.commandDefaultPattern(
                        () -> LEDPattern.solid(Leds.getAllianceColor()).blink(Seconds.of(1.0))));

        new Trigger(grabber::reverseSensorTripped)
                .and(DriverStation::isEnabled)
                .whileTrue(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kPurple))
                        .withTimeout(0.5)
                        .andThen(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kPurple))));
        new Trigger(grabber::forwardSensorTripped)
                .and(DriverStation::isEnabled)
                .whileTrue(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kYellow)));
        new Trigger(grabber::hasAlgae)
                .and(DriverStation::isEnabled)
                .whileTrue(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kAquamarine))
                        .withTimeout(0.5)
                        .andThen(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kAquamarine))));
        new Trigger(intake::hasCoral)
                .and(DriverStation::isEnabled)
                .whileTrue(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kOrangeRed)));

        MatchTriggers.endgame()
                .onTrue(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kWhite))
                        .withTimeout(1.5));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }
}

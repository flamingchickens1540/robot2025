package org.team1540.robot2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.ButtonBoard;
import org.team1540.robot2025.util.JoystickUtil;
import org.team1540.robot2025.util.MatchTriggers;
import org.team1540.robot2025.util.auto.LoggedAutoChooser;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);
    private final ButtonBoard buttonBoard = new ButtonBoard(2);
    private final CommandXboxController prankster = new CommandXboxController(3);

    private final Drivetrain drivetrain;
    private final AprilTagVision aprilTagVision;
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
                            ReefBranch.E, ReefHeight.L4, drivetrain, superstructure));
            driver.b().whileTrue(AutoScoreCommands.alignToBargeAndScore(drivetrain, superstructure));
        }

        if (Constants.isTuningMode()) {
            prankster.a().onTrue(Commands.runOnce(grabber::toggleGrabberOverride));

            prankster.y().onTrue(Commands.runOnce(intake::togglePivotOverride));
            prankster.x().onTrue(Commands.runOnce(intake::toggleRollerOverride));
            prankster.b().onTrue(Commands.runOnce(intake::toggleFunnelOverride));

            prankster.rightTrigger().onTrue(Commands.runOnce(elevator::toggleElevatorOverride));

            prankster.leftTrigger().onTrue(Commands.runOnce(drivetrain::toggleDrivetrainOverride));

            prankster.rightTrigger().onTrue(Commands.runOnce(arm::toggleArmOverride));

            prankster.leftTrigger().onTrue(Commands.runOnce(climber::toggleClimberOverride));
        }

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
        driver.rightStick()
                .and(buttonBoard.flexTrue())
                .whileTrue(Commands.waitUntil(driver.leftBumper().or(driver.rightBumper()))
                        .andThen(AutoAlignCommands.alignToNearestFace(drivetrain, driver.rightBumper())));

        driver.leftTrigger()
                .and(buttonBoard.branchHeightAt(ReefHeight.L1).negate())
                .whileTrue(superstructure.coralGroundIntake())
                .onFalse(superstructure.stow());
        driver.leftTrigger()
                .and(buttonBoard.branchHeightAt(ReefHeight.L1))
                .whileTrue(superstructure.coralGroundIntakeL1())
                .onFalse(superstructure.stow());

        driver.leftBumper()
                .whileTrue(superstructure.algaeIntake())
                .onFalse(superstructure.stow().unless(drivetrain::isAutoAligning));

        driver.rightTrigger().onTrue(superstructure.score());

        climber.setDefaultCommand(climber.climbCommand(() -> JoystickUtil.smartDeadzone(copilot.getRightY(), 0.1)));

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
        copilot.b()
                .onTrue(drivetrain
                        .teleopDriveWithHeadingCommand(
                                driver.getHID(),
                                () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.k180deg),
                                () -> true)
                        .alongWith(Commands.waitUntil(() -> Math.abs(RobotState.getInstance()
                                                .getRobotRotation()
                                                .minus(AllianceFlipUtil.maybeReverseRotation(Rotation2d.k180deg))
                                                .getDegrees())
                                        < 10)
                                .andThen(superstructure.net())));
        copilot.povLeft().onTrue(superstructure.processor());
        copilot.povDown().whileTrue(superstructure.coralIntakeEject()).onFalse(superstructure.stow());

        for (ButtonBoard.ReefButton button : ButtonBoard.ReefButton.values()) {
            for (ReefHeight height : ReefHeight.values()) {
                buttonBoard
                        .branchFaceAt(button)
                        .and(buttonBoard.branchHeightAt(height))
                        .and(buttonBoard.flexFalse())
                        .and(driver.rightStick())
                        .whileTrue(AutoScoreCommands.alignToBranchAndScore(
                                buttonBoard.reefButtonToBranch(button), height, drivetrain, superstructure));
            }
            buttonBoard
                    .branchFaceAt(button)
                    .and(buttonBoard.flexFalse())
                    .and(driver.rightBumper())
                    .whileTrue(AutoScoreCommands.alignToFaceAndDealgify(
                            buttonBoard.reefButtonToBranch(button).face, drivetrain, superstructure));
        }
    }

    private void configureAutoRoutines() {
        autoChooser.addCmd("Zero mechanisms", superstructure::zeroCommand);
        //        autoChooser.addRoutine("Right 3 Piece Lollipop", autos::right3PieceLollipop);
        //        autoChooser.addRoutine("Left 3 Piece Lollipop", autos::left3PieceLollipop);
        autoChooser.addRoutine("Right 3 Piece Sweep", autos::right3PieceSweep);
        autoChooser.addRoutine("Left 3 Piece Sweep", autos::left3PieceSweep);
        autoChooser.addRoutine("Center 1 Piece Barge", autos::center1PieceBarge);
        autoChooser.addRoutine("Center 1 Piece Processor", autos::center1PieceProcessor);
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
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            addPeriodicCallback(SimState.getInstance()::update, "Simulation update");
        }
    }

    private void addPeriodicCallback(Runnable callback, String name) {
        CommandScheduler.getInstance()
                .schedule(Commands.run(callback).withName(name).ignoringDisable(true));
    }

    private void configureLEDBindings() {
        // RobotModeTriggers.disabled().whileFalse(leds.viewFull.showRSLState());
        RobotModeTriggers.disabled()
                .onTrue(Commands.runOnce(
                        () -> leds.viewFull.setDefaultPattern(CustomLEDPatterns.movingRainbow(Hertz.of(0.2)))));
        RobotModeTriggers.autonomous()
                .onTrue(Commands.runOnce(
                        () -> leds.viewFull.setDefaultPattern(LEDPattern.solid(Leds.getAllianceColor()))));
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> leds.viewFull.setDefaultCommand(leds.viewFull.commandShowPattern(
                        () -> LEDPattern.solid(Leds.getAllianceColor()).blink(Seconds.of(1.0))))))
                .onFalse(Commands.runOnce(leds.viewFull::removeDefaultCommand));

        new Trigger(grabber::reverseSensorTripped)
                .and(DriverStation::isEnabled)
                .whileTrue(leds.viewFull
                        .commandShowPattern(CustomLEDPatterns.strobe(Color.kPurple))
                        .withTimeout(0.5)
                        .andThen(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kPurple))));
        new Trigger(grabber::forwardSensorTripped)
                .and(DriverStation::isEnabled)
                .whileTrue(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kYellow)));
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

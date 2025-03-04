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
import org.team1540.robot2025.util.ButtonBoard;
import org.team1540.robot2025.util.JoystickUtil;
import org.team1540.robot2025.util.auto.LoggedAutoChooser;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);
    private final ButtonBoard buttonBoard = new ButtonBoard(2);

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
    }

    private void configureButtonBindings() {
        // Sim testing binding
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            driver.x()
                    .onTrue(AutoScoreCommands.alignToBranchAndScore(
                            FieldConstants.ReefBranch.E,
                            ReefHeight.L3,
                            driver.rightTrigger(),
                            drivetrain,
                            superstructure));
        }

        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID(), () -> true));
        driver.back().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        driver.leftStick().onTrue(superstructure.stow());
        driver.rightStick()
                .and(buttonBoard.flexTrue())
                .whileTrue(Commands.waitUntil(driver.leftBumper().or(driver.rightBumper()))
                        .andThen(AutoAlignCommands.alignToNearestFace(drivetrain, driver.rightBumper())));

        driver.leftTrigger()
                .and(buttonBoard.branchHeightAt(ReefHeight.L1).negate())
                //                .and(() -> !grabber.hasAlgae())
                .whileTrue(superstructure.coralGroundIntake())
                .onFalse(superstructure.stow());
        driver.leftTrigger()
                .and(buttonBoard.branchHeightAt(ReefHeight.L1))
                //                .or(grabber::hasAlgae)
                .whileTrue(superstructure.coralGroundIntakeL1())
                .onFalse(superstructure.stow());

        climber.setDefaultCommand(climber.manualCommand(() -> JoystickUtil.smartDeadzone(copilot.getRightY(), 0.1)));

        copilot.start().whileTrue(superstructure.zeroCommand());
        copilot.back()
                .toggleOnTrue(elevator.manualCommand(() -> 0.5 * -JoystickUtil.smartDeadzone(copilot.getLeftY(), 0.1)));
        copilot.rightTrigger().whileTrue(superstructure.coralGroundIntake()).onFalse(superstructure.stow());
        copilot.leftTrigger().whileTrue(superstructure.algaeIntake()).onFalse(superstructure.stow());
        copilot.leftBumper().onTrue(superstructure.dealgifyHigh());
        copilot.rightBumper().onTrue(superstructure.dealgifyLow());

        copilot.y().onTrue(superstructure.L4(driver.rightTrigger(), RobotState.getInstance()::shouldReverseCoral));
        copilot.x().onTrue(superstructure.L3(driver.rightTrigger(), RobotState.getInstance()::shouldReverseCoral));
        copilot.a().onTrue(superstructure.L2(driver.rightTrigger(), RobotState.getInstance()::shouldReverseCoral));
        copilot.povRight().onTrue(superstructure.L1(driver.rightTrigger()));
        copilot.b().onTrue(superstructure.net(driver.rightTrigger()));

        copilot.povLeft().onTrue(superstructure.processor(driver.rightTrigger()));
        copilot.povDown().whileTrue(superstructure.coralIntakeEject()).onFalse(superstructure.stow());
        copilot.rightStick().onTrue(superstructure.stow());

        for (ButtonBoard.ReefButton button : ButtonBoard.ReefButton.values()) {
            for (ReefHeight height : ReefHeight.values()) {
                buttonBoard
                        .branchFaceAt(button)
                        .and(buttonBoard.branchHeightAt(height))
                        .and(buttonBoard.flexFalse())
                        .and(driver.rightStick())
                        .whileTrue(AutoScoreCommands.alignToBranchAndScore(
                                buttonBoard.reefButtonToBranch(button),
                                height,
                                driver.rightTrigger(),
                                drivetrain,
                                superstructure));
            }
            buttonBoard
                    .branchFaceAt(button)
                    .and(buttonBoard.flexFalse())
                    .and(driver.rightBumper())
                    .whileTrue(AutoScoreCommands.alignToFaceAndDealgify(
                            buttonBoard.reefButtonToBranch(button).face, drivetrain, superstructure));
        }

        new Trigger(grabber::reverseSensorTripped)
                .whileTrue(leds.viewFull
                        .commandShowPattern(LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.05)))
                        .withTimeout(0.5)
                        .andThen(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kPurple))));
        new Trigger(grabber::forwardSensorTripped)
                .whileTrue(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kYellow)));
        new Trigger(intake::hasCoral).whileTrue(leds.viewFull.commandShowPattern(LEDPattern.solid(Color.kOrange)));
    }

    private void configureAutoRoutines() {
        autoChooser.addCmd("Zero mechanisms", superstructure::zeroCommand);
        autoChooser.addRoutine("Right 3 Piece Lollipop", autos::right3PieceLollipop);
        if (Constants.isTuningMode()) {
            autoChooser.addCmd("Drive FF Characterization", drivetrain::feedforwardCharacterization);
            autoChooser.addCmd("Drive Wheel Radius Characterization", drivetrain::wheelRadiusCharacterization);
            autoChooser.addCmd("Elevator FF Characterization", elevator::feedforwardCharacterizationCommand);
        }
    }

    private void configureRobotModeTriggers() {
        //        RobotModeTriggers.disabled().whileFalse(leds.viewFull.showRSLState());
        RobotModeTriggers.disabled()
                .whileTrue(leds.viewFull.commandShowPattern(
                        CustomLEDPatterns.movingRainbow(Value.one().div(Second.of(5)))));
        RobotModeTriggers.autonomous()
                .onTrue(Commands.runOnce(() -> leds.viewFull.setDefaultCommand(
                        leds.viewFull.commandShowPattern(() -> LEDPattern.solid(Leds.getAllianceColor())))));
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> leds.viewFull.setDefaultCommand(leds.viewFull.commandShowPattern(
                        () -> LEDPattern.solid(Leds.getAllianceColor()).breathe(Seconds.of(3))))));
        RobotModeTriggers.teleop()
                .and(DriverStation::isFMSAttached)
                .onTrue(Commands.runOnce(drivetrain::zeroFieldOrientation));
    }

    private void configurePeriodicCallbacks() {
        addPeriodicCallback(AlertManager.getInstance()::update, "AlertManager update");
        addPeriodicCallback(MechanismVisualizer.getInstance()::update, "MechanismVisualizer update");
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            addPeriodicCallback(SimState.getInstance()::update, "Simulation update");
        }
    }

    private void addPeriodicCallback(Runnable callback, String name) {
        CommandScheduler.getInstance()
                .schedule(Commands.run(callback).withName(name).ignoringDisable(true));
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

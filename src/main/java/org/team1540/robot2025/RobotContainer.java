package org.team1540.robot2025;

import static edu.wpi.first.units.Units.Seconds;
import static org.team1540.robot2025.FieldConstants.ReefHeight.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team1540.robot2025.autos.Autos;
import org.team1540.robot2025.commands.AutoAlignCommands;
import org.team1540.robot2025.services.AlertManager;
import org.team1540.robot2025.services.MechanismVisualizer;
import org.team1540.robot2025.subsystems.Superstructure;
import org.team1540.robot2025.subsystems.Superstructure.SuperstructureState;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.climber.Climber;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.CoralIntake;
import org.team1540.robot2025.subsystems.leds.Leds;
import org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVision;
import org.team1540.robot2025.util.ButtonBoard;
import org.team1540.robot2025.util.ButtonBoard.ReefButton;
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
    private final CoralIntake coralIntake;
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
                coralIntake = CoralIntake.createReal();
                grabber = Grabber.createReal();
                climber = Climber.createDummy();
                break;
            case SIM:
                // Simulation, instantiate physics sim IO implementations
                drivetrain = Drivetrain.createSim();
                aprilTagVision = AprilTagVision.createSim();
                elevator = Elevator.createSim();
                arm = Arm.createSim();
                coralIntake = CoralIntake.createSim();
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
                coralIntake = CoralIntake.createDummy();
                grabber = Grabber.createDummy();
                climber = Climber.createDummy();
        }
        superstructure = new Superstructure(elevator, arm, coralIntake, grabber);
        autos = new Autos(drivetrain, superstructure);

        configureButtonBindings();
        configureAutoRoutines();
        configureRobotModeTriggers();
        configurePeriodicCallbacks();
    }

    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID(), () -> true));
        driver.back().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        driver.rightStick().onTrue(superstructure.commandToState(SuperstructureState.STOW));
        driver.leftStick()
                .whileTrue(Commands.waitUntil(driver.leftBumper().or(driver.rightBumper()))
                        .andThen(AutoAlignCommands.alignToNearestFace(drivetrain, driver.rightBumper())));

        copilot.start().whileTrue(superstructure.zeroCommand());
        copilot.back()
                .toggleOnTrue(elevator.manualCommand(() -> 0.5 * -JoystickUtil.smartDeadzone(copilot.getLeftY(), 0.1)));
        copilot.rightTrigger()
                .whileTrue(superstructure.coralGroundIntake())
                .onFalse(superstructure.commandToState(SuperstructureState.STOW));
        copilot.leftTrigger()
                .whileTrue(superstructure.algaeIntake())
                .onFalse(superstructure.commandToState(SuperstructureState.STOW));
        copilot.leftBumper().onTrue(superstructure.dealgifyHigh());
        copilot.rightBumper().onTrue(superstructure.dealgifyLow());

        copilot.y().onTrue(superstructure.L4(driver.rightTrigger()));
        copilot.x().onTrue(superstructure.L3(driver.rightTrigger()));
        copilot.a().onTrue(superstructure.L2(driver.rightTrigger()));
        copilot.povRight().onTrue(superstructure.L1(driver.rightTrigger()));
        copilot.b().onTrue(superstructure.net());

        copilot.povLeft().onTrue(superstructure.processor(driver.rightTrigger()));
        copilot.povDown()
                .whileTrue(superstructure.coralIntakeEject())
                .onFalse(superstructure.commandToState(SuperstructureState.STOW));

        buttonBoard
                .branchFaceAt(ReefButton.A)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.A, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.B)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.B, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.C)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.C, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.D)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.D, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.E)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.E, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.F)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.F, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.G)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.G, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.H)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.H, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.I)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.I, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.J)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.J, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.K)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.K, drivetrain));
        buttonBoard
                .branchFaceAt(ReefButton.L)
                .and(driver.leftStick())
                .whileTrue(AutoAlignCommands.alignToBranch(FieldConstants.ReefBranch.L, drivetrain));
        //        buttonBoard.branchFaceAt(ReefButton.A).onTrue(Commands.print("A"));
        //        buttonBoard.branchFaceAt(ReefButton.B).onTrue(Commands.print("B"));
        //        buttonBoard.branchFaceAt(ReefButton.C).onTrue(Commands.print("C"));
        //        buttonBoard.branchFaceAt(ReefButton.D).onTrue(Commands.print("D"));
        //        buttonBoard.branchFaceAt(ReefButton.E).onTrue(Commands.print("E"));
        //        buttonBoard.branchFaceAt(ReefButton.F).onTrue(Commands.print("F"));
        //        buttonBoard.branchFaceAt(ReefButton.G).onTrue(Commands.print("G"));
        //        buttonBoard.branchFaceAt(ReefButton.H).onTrue(Commands.print("H"));
        //        buttonBoard.branchFaceAt(ReefButton.I).onTrue(Commands.print("I"));
        //        buttonBoard.branchFaceAt(ReefButton.J).onTrue(Commands.print("J"));
        //        buttonBoard.branchFaceAt(ReefButton.K).onTrue(Commands.print("K"));
        //        buttonBoard.branchFaceAt(ReefButton.L).onTrue(Commands.print("L"));
    }

    private void configureAutoRoutines() {
        autoChooser.addCmd("Zero mechanisms", superstructure::zeroCommand);
        if (Constants.isTuningMode()) {
            autoChooser.addCmd("Drive FF Characterization", drivetrain::feedforwardCharacterization);
            autoChooser.addCmd("Drive Wheel Radius Characterization", drivetrain::wheelRadiusCharacterization);
            autoChooser.addCmd("Elevator FF Characterization", elevator::feedforwardCharacterizationCommand);
        }
    }

    private void configureRobotModeTriggers() {
        RobotModeTriggers.disabled().whileFalse(leds.viewFull.showRSLState());
        RobotModeTriggers.autonomous()
                .whileTrue(leds.viewTop.commandShowPattern(() -> LEDPattern.solid(Leds.getAllianceColor())));
        RobotModeTriggers.teleop()
                .whileTrue(leds.viewTop.commandShowPattern(
                        () -> LEDPattern.solid(Leds.getAllianceColor()).breathe(Seconds.of(3))));
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

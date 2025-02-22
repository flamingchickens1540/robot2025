package org.team1540.robot2025;

import static edu.wpi.first.units.Units.Seconds;

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
import org.team1540.robot2025.services.AlertManager;
import org.team1540.robot2025.services.MechanismVisualizer;
import org.team1540.robot2025.subsystems.Superstructure;
import org.team1540.robot2025.subsystems.arm.Arm;
import org.team1540.robot2025.subsystems.climber.Climber;
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.CoralIntake;
import org.team1540.robot2025.subsystems.leds.Leds;
import org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVision;
import org.team1540.robot2025.util.JoystickUtil;
import org.team1540.robot2025.util.auto.LoggedAutoChooser;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

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

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
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
                aprilTagVision = AprilTagVision.createDummy();
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
        climber.setDefaultCommand(climber.manualCommand(() -> -copilot.getRightY()));
        driver.back().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        // Test Holding Algae
        //        LoggedTunableNumber grabberPercent = new LoggedTunableNumber("Grabber/Percent", 0.25);
        //        driver.b().whileTrue(arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE));
        //        driver.a().whileTrue(Commands.runOnce(() -> grabber.setPercent(grabberPercent.getAsDouble())));

        // Ground Algae
        //        LoggedTunableNumber grabberPercent = new LoggedTunableNumber("Grabber/Percent", 0.25);
        //        driver.a().whileTrue(elevator.commandToSetpoint(Elevator.ElevatorState.FLOOR_ALGAE));
        //        driver.b().whileTrue(arm.commandToSetpoint(Arm.ArmState.FLOOR_ALGAE));
        //        driver.y().whileTrue(elevator.commandToSetpoint(Elevator.ElevatorState.L3));
        //        driver.rightBumper().whileTrue(grabber.commandRun(grabberPercent.getAsDouble()));

        // Reverse Scoring Setpoints
        //        LoggedTunableNumber grabberPercent = new LoggedTunableNumber("Grabber/Percent", -0.5);
        //        driver.a().whileTrue(arm.commandToSetpoint(Arm.ArmState.SCORE_REVERSE));
        //        driver.b().whileTrue(elevator.commandToSetpoint(Elevator.ElevatorState.L2));
        //        driver.y().whileTrue(elevator.commandToSetpoint(Elevator.ElevatorState.L3));
        //        driver.rightBumper().whileTrue(grabber.commandRun(grabberPercent.getAsDouble()));

        // Full Driver Controls

        driver.leftTrigger().whileTrue(superstructure.coralGroundIntake());

        driver.leftBumper().whileTrue(superstructure.dealgifyHigh());
        driver.rightBumper().whileTrue(superstructure.dealgifyLow());
        driver.leftStick().whileTrue(superstructure.algaeIntake());

        driver.back().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        driver.y().onTrue(superstructure.L4(driver.rightTrigger()));
        driver.x().onTrue(superstructure.L3(driver.rightTrigger()));
        driver.b().onTrue(superstructure.L2(driver.rightTrigger()));
        driver.a().onTrue(superstructure.net());
        driver.povRight().onTrue(superstructure.L1(driver.rightTrigger()));

        driver.povDown().whileTrue(superstructure.processor(driver.rightTrigger()));

        driver.rightStick().whileTrue(superstructure.commandToState(Superstructure.SuperstructureState.STOW));

        copilot.x().whileTrue(Commands.sequence(arm.commandToSetpoint(Arm.ArmState.STOW), elevator.zeroCommand()));
        copilot.b().whileTrue(coralIntake.zeroCommand());

        copilot.y()
                .toggleOnTrue(elevator.manualCommand(() -> 0.5 * -JoystickUtil.smartDeadzone(copilot.getLeftY(), 0.1)));
    }

    private void configureAutoRoutines() {
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

package org.team1540.robot2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.grabber.Grabber;
import org.team1540.robot2025.subsystems.intake.CoralIntake;
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

    private final Superstructure superstructure;

    private final Autos autos;
    private final LoggedAutoChooser autoChooser = new LoggedAutoChooser("Auto Chooser");

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain = Drivetrain.createReal();
                aprilTagVision = AprilTagVision.createDummy();
                elevator = Elevator.createReal();
                arm = Arm.createReal();
                coralIntake = CoralIntake.createReal();
                grabber = Grabber.createReal();
                break;
            case SIM:
                // Simulation, instantiate physics sim IO implementations
                drivetrain = Drivetrain.createSim();
                aprilTagVision = AprilTagVision.createDummy();
                elevator = Elevator.createSim();
                arm = Arm.createSim();
                coralIntake = CoralIntake.createSim();
                grabber = Grabber.createSim();

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
        //                driver.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        //                driver.y().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        copilot.y().onTrue(Commands.runOnce(() -> elevator.resetPosition(0.0)));
        copilot.x().toggleOnTrue(elevator.manualCommand(() -> -JoystickUtil.smartDeadzone(copilot.getLeftY(), 0.1)));
        //        copilot.a().whileTrue(elevator.setpointCommand(Elevator.ElevatorState.L1));
        //        copilot.a().whileTrue(elevator.setpointCommand(Elevator.ElevatorState.L3));

        //        copilot.a().whileTrue(arm.commandToSetpoint(Arm.ArmState.STOW));
        //        copilot.b().whileTrue(arm.commandToSetpoint(Arm.ArmState.INTAKE));
        //                copilot.a().whileTrue(grabber.commandRun(0.5));

        //        copilot.b()
        //                .whileTrue(Commands.parallel(
        //                        elevator.commandToSetpoint(Elevator.ElevatorState.L4),
        //                        arm.commandToSetpoint(Arm.ArmState.L4_SCORE)));
        //        //                        Commands.waitUntil(() -> arm.getPosition().getDegrees()
        //        //                                        > Arm.ArmState.SCORE.position().getDegrees() - 5)
        //        //                                .andThen(grabber.commandRun(0.5))));
        //        copilot.a()
        //                .whileTrue(Commands.parallel(
        //                        elevator.commandToSetpoint(Elevator.ElevatorState.L4),
        //                        arm.commandToSetpoint(Arm.ArmState.L4_SCORE_REVERSE)));
        //        copilot.b()
        //                .whileTrue(Commands.parallel(
        //                        elevator.setpointCommand(Elevator.ElevatorState.L2),
        //                        arm.commandToSetpoint(Arm.ArmState.LOW_SCORE)));

        copilot.rightBumper().whileTrue(grabber.commandRun(0.5));
        copilot.leftBumper().whileTrue(grabber.commandRun(-0.2));
        //        copilot.a()
        //                .whileTrue(Commands.sequence(
        //                        elevator.setpointCommand(Elevator.ElevatorState.L4),
        // arm.commandToSetpoint(Arm.ArmState.STOW)));
        //        copilot.rightTrigger()
        //                .whileTrue(Commands.parallel(
        //                        Commands.sequence(
        //                                arm.commandToSetpoint(Arm.ArmState.L4_SCORE),
        //                                elevator.commandToSetpoint(Elevator.ElevatorState.L4)),
        //                        Commands.sequence(
        //                                grabber.commandRun(0.25).until(() -> elevator.getPosition() > 1.25),
        //                                grabber.commandRun(-1))));

        //        driver.y().whileTrue(superstructure.L4(driver.rightTrigger()));
        //        driver.x().whileTrue(superstructure.L3(driver.rightTrigger()));
        //        driver.b().whileTrue(superstructure.L2(driver.rightTrigger()));
        //                driver.a().whileTrue(superstructure.L1(driver.rightTrigger()));

        //        driver.a().whileTrue(arm.commandToSetpoint(Arm.ArmState.L1_SCORE));
        //        driver.b().whileTrue(arm.commandToSetpoint(Arm.ArmState.STOW_ALGAE));
        //        driver.b().whileTrue(elevator.commandToSetpoint(Elevator.ElevatorState.L2));
        //        driver.y().whileTrue(elevator.commandToSetpoint(Elevator.ElevatorState.L3));
        //        driver.x().whileTrue(arm.commandToSetpoint(Arm.ArmState.STOW).andThen(elevator.zeroCommand()));

        driver.a().whileTrue(coralIntake.commandToSetpoint(CoralIntake.CoralIntakeState.INTAKE));
        driver.b().whileTrue(coralIntake.commandToSetpoint(CoralIntake.CoralIntakeState.STOW));
        driver.x().whileTrue(Commands.runOnce(() -> coralIntake.setFunnelVoltage(6)));
        driver.y().whileTrue(arm.commandToSetpoint(Arm.ArmState.INTAKE).andThen(grabber.commandRun(0.5)));

        driver.leftTrigger().whileTrue(superstructure.net());
        driver.leftBumper().whileTrue(superstructure.processor(driver.rightTrigger()));
        driver.rightBumper().whileTrue(superstructure.dealgifyHigh());

        driver.back().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        driver.rightStick().whileTrue(superstructure.stow());
        driver.leftStick().whileTrue(superstructure.coralGroundIntake());
    }

    private void configureAutoRoutines() {
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

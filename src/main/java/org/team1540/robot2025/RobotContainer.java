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
import org.team1540.robot2025.subsystems.drive.Drivetrain;
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.auto.LoggedAutoChooser;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);

    private final Drivetrain drivetrain;

    private final Autos autos;
    private final LoggedAutoChooser autoChooser = new LoggedAutoChooser("Auto Chooser");

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain = Drivetrain.createReal();
                break;
            case SIM:
                // Simulation, instantiate physics sim IO implementations
                drivetrain = Drivetrain.createSim();

                RobotState.getInstance().resetPose(new Pose2d(3.0, 3.0, Rotation2d.kZero));
                break;
            default:
                // Replayed robot, disable IO implementations
                drivetrain = Drivetrain.createDummy();
        }
        autos = new Autos(drivetrain);

        configureButtonBindings();
        configureAutoRoutines();
        configureRobotModeTriggers();
        configurePeriodicCallbacks();
    }

    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID(), () -> true));
        driver.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.y().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        driver.povUp()
                .onTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(), () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kZero), () -> true));
        driver.povLeft()
                .onTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(),
                        () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kCCW_90deg),
                        () -> true));
        driver.povDown()
                .onTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(), () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.k180deg), () -> true));
        driver.povRight()
                .onTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(),
                        () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kCW_90deg),
                        () -> true));
    }

    private void configureAutoRoutines() {
        if (Constants.isTuningMode()) {
            autoChooser.addCmd("Drive FF Characterization", drivetrain::feedforwardCharacterization);
            autoChooser.addCmd("Drive Wheel Radius Characterization", drivetrain::wheelRadiusCharacterization);
        }
    }

    private void configureRobotModeTriggers() {
        RobotModeTriggers.teleop()
                .and(DriverStation::isFMSAttached)
                .onTrue(Commands.runOnce(drivetrain::zeroFieldOrientation));
        RobotModeTriggers.teleop()
                .or(RobotModeTriggers.autonomous())
                .onTrue(Commands.runOnce(() -> drivetrain.setBrakeMode(true)));
        RobotModeTriggers.disabled()
                .whileTrue(Commands.waitSeconds(5.0).andThen(Commands.runOnce(() -> drivetrain.setBrakeMode(false))));
    }

    private void configurePeriodicCallbacks() {
        CommandScheduler.getInstance()
                .schedule(Commands.run(AlertManager.getInstance()::update)
                        .withName("AlertManager update")
                        .ignoringDisable(true));

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            CommandScheduler.getInstance()
                    .schedule(Commands.run(SimState.getInstance()::update)
                            .withName("Simulation update")
                            .ignoringDisable(true));
        }
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

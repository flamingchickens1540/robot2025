package org.team1540.robot2025.subsystems.drive;

import static org.team1540.robot2025.subsystems.drive.DrivetrainConstants.*;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.Robot;
import org.team1540.robot2025.RobotState;
import org.team1540.robot2025.SimState;
import org.team1540.robot2025.commands.CharacterizationCommands;
import org.team1540.robot2025.generated.TunerConstants;
import org.team1540.robot2025.util.*;
import org.team1540.robot2025.util.swerve.AutoAlignController;
import org.team1540.robot2025.util.swerve.TrajectoryController;

public class Drivetrain extends SubsystemBase {
    private static boolean hasInstance;
    static final Lock odometryLock = new ReentrantLock();

    private static final LoggedTunableNumber translationKP = new LoggedTunableNumber("Drivetrain/Translation/kP", 6.0);
    private static final LoggedTunableNumber translationKI = new LoggedTunableNumber("Drivetrain/Translation/kI", 0.0);
    private static final LoggedTunableNumber translationKD = new LoggedTunableNumber("Drivetrain/Translation/kD", 0.0);

    private static final LoggedTunableNumber headingKP = new LoggedTunableNumber("Drivetrain/Heading/kP", 4.0);
    private static final LoggedTunableNumber headingKI = new LoggedTunableNumber("Drivetrain/Heading/kI", 0.0);
    private static final LoggedTunableNumber headingKD = new LoggedTunableNumber("Drivetrain/Heading/kD", 0.0);

    private static final LoggedTunableNumber autoAlignLinearSpeedFactor =
            new LoggedTunableNumber("AutoAlign/LinearSpeedFactor", 0.75);
    private static final LoggedTunableNumber autoAlignLinearAccelFactor =
            new LoggedTunableNumber("AutoAlign/LinearAccelFactor", 0.5);
    private static final LoggedTunableNumber autoAlignRotationSpeedFactor =
            new LoggedTunableNumber("AutoAlign/RotationSpeedFactor", 0.5);
    private static final LoggedTunableNumber autoAlignRotationAccelFactor =
            new LoggedTunableNumber("AutoAlign/RotationAccelFactor", 0.5);

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SwerveDriveKinematics kinematics = RobotState.getInstance().getKinematics();

    private Rotation2d fieldOrientationOffset = Rotation2d.kZero;

    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4]; // For odometry delta filtering
    private double lastOdometryUpdateTime = 0.0;

    @AutoLogOutput(key = "Drivetrain/DesiredSpeeds")
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private SwerveSetpoint lastSetpoint;
    private final SwerveSetpointGenerator setpointGenerator =
            new SwerveSetpointGenerator(ROBOT_CONFIG, MAX_STEER_SPEED_RAD_PER_SEC);

    private boolean isAutoAligning = false;
    private final Debouncer atAutoAlignGoalDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private boolean isFFCharacterizing = false;
    private double ffCharacterizationInput = 0.0;

    private final TrajectoryController trajectoryController = new TrajectoryController(
            translationKP.get(),
            translationKI.get(),
            translationKD.get(),
            headingKP.get(),
            headingKI.get(),
            headingKD.get());

    private final AutoAlignController autoAlignController = new AutoAlignController(
            translationKP.get(),
            translationKI.get(),
            translationKD.get(),
            headingKP.get(),
            headingKI.get(),
            headingKD.get(),
            MAX_LINEAR_SPEED_MPS * autoAlignLinearSpeedFactor.get(),
            MAX_LINEAR_ACCEL_MPS2 * autoAlignLinearAccelFactor.get(),
            MAX_ANGULAR_SPEED_RAD_PER_SEC * autoAlignRotationSpeedFactor.get(),
            MAX_ANGULAR_ACCEL_RAD_PER_SEC2 * autoAlignRotationAccelFactor.get());

    private final ProfiledPIDController headingController = new ProfiledPIDController(
            headingKP.get(),
            headingKI.get(),
            headingKD.get(),
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RAD_PER_SEC, MAX_ANGULAR_ACCEL_RAD_PER_SEC2));

    private final Alert gyroDisconnected = new Alert("Gyro is disconnected", Alert.AlertType.kError);

    public Drivetrain(
            GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        if (hasInstance) throw new IllegalStateException("Instance of drivetrain already exists");
        hasInstance = true;

        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, Module.MountPosition.FL, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, Module.MountPosition.FR, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, Module.MountPosition.BL, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, Module.MountPosition.BR, TunerConstants.BackRight);

        for (int i = 0; i < 4; i++) {
            lastModulePositions[i] = modules[i].getPosition();
        }

        headingController.setTolerance(Math.toRadians(1.0));
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        AutoBuilder.configure(
                RobotState.getInstance()::getEstimatedPose,
                RobotState.getInstance()::resetPose,
                RobotState.getInstance()::getRobotVelocity,
                this::runVelocity,
                new PPHolonomicDriveController(
                        new PIDConstants(translationKP.get(), translationKI.get(), translationKD.get()),
                        new PIDConstants(headingKP.get(), headingKI.get(), headingKD.get())),
                ROBOT_CONFIG,
                AllianceFlipUtil::shouldFlip,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                poseList -> RobotState.getInstance().setActiveTrajectory(poseList.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(RobotState.getInstance()::setTrajectoryTarget);

        lastSetpoint = new SwerveSetpoint(
                desiredSpeeds,
                getModuleStates(),
                new DriveFeedforwards(new double[4], new double[4], new double[4], new double[4], new double[4]));

        // Start odometry thread
        OdometryThread.getInstance().start();
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();

        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        for (Module module : modules) module.periodic();
        odometryLock.unlock();

        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        int rejectedSamples = 0;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
            }

            // Filter odometry data based on wheel deltas
            boolean acceptMeasurement = true;
            double dt = sampleTimestamps[i] - lastOdometryUpdateTime;
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                double velocity = moduleDeltas[moduleIndex].distanceMeters / dt;
                double turnVelocity = modulePositions[moduleIndex]
                                .angle
                                .minus(lastModulePositions[moduleIndex].angle)
                                .getRadians()
                        / dt;
                if (Math.abs(velocity) > MAX_LINEAR_SPEED_MPS * 2
                        || Math.abs(turnVelocity) > MAX_STEER_SPEED_RAD_PER_SEC * 2) {
                    acceptMeasurement = false;
                    break;
                }
            }
            // Accept measurements if delta is not too large
            if (acceptMeasurement) {
                if (gyroInputs.connected) rawGyroRotation = gyroInputs.odometryYawPositions[i];
                else {
                    Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
                    rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
                }
                RobotState.getInstance().addOdometryObservation(modulePositions, rawGyroRotation, sampleTimestamps[i]);
                lastModulePositions = modulePositions;
                lastOdometryUpdateTime = sampleTimestamps[i];
            } else {
                rejectedSamples++;
            }
        }
        Logger.recordOutput("Odometry/RejectedSamples", rejectedSamples);

        // Update robot velocities
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        speeds.omegaRadiansPerSecond =
                gyroInputs.connected ? gyroInputs.yawVelocityRadPerSec : speeds.omegaRadiansPerSecond;
        RobotState.getInstance().addVelocityData(speeds);

        if (DriverStation.isEnabled()) {
            // Run modules based on current drive mode
            if (isFFCharacterizing) {
                for (Module module : modules) module.runCharacterization(ffCharacterizationInput);
            } else {
                SwerveModuleState[] setpointStates;
                if (OPTIMIZE_SETPOINTS) {
                    SwerveSetpoint newSetpoint =
                            setpointGenerator.generateSetpoint(lastSetpoint, desiredSpeeds, Constants.LOOP_PERIOD_SECS);
                    setpointStates = newSetpoint.moduleStates();
                    lastSetpoint = newSetpoint;
                } else {
                    setpointStates = kinematics.toSwerveModuleStates(
                            ChassisSpeeds.discretize(desiredSpeeds, Constants.LOOP_PERIOD_SECS));
                }

                for (int i = 0; i < 4; i++) {
                    modules[i].runSetpoint(setpointStates[i]);
                }
                Logger.recordOutput("Drivetrain/SwerveStates/Setpoints", setpointStates);
            }
        } else {
            for (Module module : modules) module.stop(); // Stop modules when disabled
            Logger.recordOutput(
                    "Drivetrain/SwerveStates/Setpoints",
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState());
        }

        // Update gyro alerts
        gyroDisconnected.set(Robot.isReal() && !gyroInputs.connected);

        // Update tunable numbers
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    trajectoryController.setTranslationPID(
                            translationKP.get(), translationKI.get(), translationKD.get());
                    autoAlignController.setTranslationPID(
                            translationKP.get(), translationKI.get(), translationKD.get());
                },
                translationKP,
                translationKI,
                translationKD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    trajectoryController.setHeadingPID(headingKP.get(), headingKI.get(), headingKD.get());
                    autoAlignController.setHeadingPID(headingKP.get(), headingKI.get(), headingKD.get());
                    headingController.setPID(headingKP.get(), headingKI.get(), headingKD.get());
                },
                headingKP,
                headingKI,
                headingKD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    autoAlignController.setTranslationConstraints(
                            autoAlignLinearSpeedFactor.get() * MAX_LINEAR_SPEED_MPS,
                            autoAlignLinearAccelFactor.get() * MAX_LINEAR_ACCEL_MPS2);
                    autoAlignController.setRotationConstraints(
                            autoAlignRotationSpeedFactor.get() * MAX_ANGULAR_SPEED_RAD_PER_SEC,
                            autoAlignRotationAccelFactor.get() * MAX_ANGULAR_ACCEL_RAD_PER_SEC2);
                },
                autoAlignLinearSpeedFactor,
                autoAlignLinearAccelFactor,
                autoAlignRotationSpeedFactor,
                autoAlignRotationAccelFactor);

        LoggedTracer.record("Drivetrain");
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    private void runVelocity(ChassisSpeeds speeds) {
        desiredSpeeds = speeds;
    }

    public void followTrajectory(SwerveSample trajectorySample) {
        RobotState.getInstance().setTrajectoryTarget(trajectorySample.getPose());
        runVelocity(trajectoryController.calculate(RobotState.getInstance().getEstimatedPose(), trajectorySample));
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        Translation2d[] modulePositions = getModuleTranslations();
        for (int i = 0; i < 4; i++) headings[i] = modulePositions[i].getAngle();
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Zeroes field-oriented drive to the direction the robot is facing
     */
    public void zeroFieldOrientationManual() {
        fieldOrientationOffset = rawGyroRotation;
    }

    /**
     * Zeroes field-oriented drive to the field based on the calculated odometry yaw
     */
    public void zeroFieldOrientation() {
        fieldOrientationOffset = rawGyroRotation.minus(
                AllianceFlipUtil.maybeFlipRotation(RobotState.getInstance().getRobotRotation()));
    }

    /**
     * Sets the brake mode of all modules
     */
    public void setBrakeMode(boolean enabled) {
        for (Module module : modules) module.setBrakeMode(enabled);
    }

    /**
     * Orients all modules forward and applies the specified voltage to the drive motors
     */
    private void runFFCharacterization(double volts) {
        ffCharacterizationInput = volts;
        isFFCharacterizing = true;
    }

    /**
     * Ends characterization and returns to default drive behavior
     */
    private void endFFCharacterization() {
        ffCharacterizationInput = 0;
        isFFCharacterizing = false;
    }

    /**
     * Returns the average velocity of each module in rot/s
     */
    private double getFFCharacterizationVelocity() {
        double driveVelocityAverage = 0;
        for (Module module : modules) driveVelocityAverage += module.getFFCharacterizationVelocity();
        return driveVelocityAverage / modules.length;
    }

    /**
     * Returns the position of each module in radians
     */
    private double[] getWheelRadiusCharacterizationPositions() {
        return Arrays.stream(modules)
                .mapToDouble(Module::getWheelRadiusCharacterizationPosition)
                .toArray();
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all the modules.
     */
    @AutoLogOutput(key = "Drivetrain/SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    @AutoLogOutput(key = "AutoAlign/IsRunning")
    public boolean isAutoAligning() {
        return isAutoAligning;
    }

    @AutoLogOutput(key = "AutoAlign/AtGoal")
    public boolean atAutoAlignGoal() {
        return atAutoAlignGoalDebounce.calculate(autoAlignController.atGoal(0.03, Rotation2d.fromDegrees(1.0)));
    }

    public Command percentDriveCommand(
            Supplier<Translation2d> linearPercent, DoubleSupplier omegaPercent, BooleanSupplier fieldRelative) {
        return Commands.run(
                        () -> {
                            var speeds = new ChassisSpeeds(
                                    linearPercent.get().getX() * MAX_LINEAR_SPEED_MPS,
                                    linearPercent.get().getY() * MAX_LINEAR_SPEED_MPS,
                                    omegaPercent.getAsDouble() * MAX_ANGULAR_SPEED_RAD_PER_SEC);
                            if (fieldRelative.getAsBoolean()) {
                                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                        speeds, rawGyroRotation.minus(fieldOrientationOffset));
                            }
                            runVelocity(speeds);
                        },
                        this)
                .finallyDo(this::stop);
    }

    public Command teleopDriveCommand(XboxController controller, BooleanSupplier fieldRelative) {
        return percentDriveCommand(
                () -> JoystickUtil.deadzonedJoystickTranslation(-controller.getLeftY(), -controller.getLeftX(), 0.1),
                () -> JoystickUtil.smartDeadzone(-controller.getRightX(), 0.1),
                fieldRelative);
    }

    public Command teleopDriveWithHeadingCommand(
            XboxController controller, Supplier<Rotation2d> heading, BooleanSupplier fieldRelative) {
        return percentDriveCommand(
                        () -> JoystickUtil.deadzonedJoystickTranslation(
                                -controller.getLeftY(), -controller.getLeftX(), 0.1),
                        () -> headingController.calculate(
                                        RobotState.getInstance()
                                                .getRobotRotation()
                                                .getRadians(),
                                        new TrapezoidProfile.State(heading.get().getRadians(), 0.0))
                                / MAX_ANGULAR_SPEED_RAD_PER_SEC,
                        fieldRelative)
                .beforeStarting(() -> headingController.reset(
                        RobotState.getInstance().getRobotRotation().getRadians(),
                        RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond))
                .alongWith(Commands.run(() -> Logger.recordOutput("Drivetrain/HeadingGoal", heading.get())))
                .until(() -> Math.abs(controller.getRightX()) >= 0.1);
    }

    public Command driveToPoseCommand(Supplier<Pose2d> goalPose, Supplier<Pose2d> poseEstimateSource) {
        return Commands.startRun(
                        () -> {
                            autoAlignController.setGoal(goalPose);
                            isAutoAligning = true;
                        },
                        () -> runVelocity(autoAlignController.calculate(
                                poseEstimateSource.get(),
                                RobotState.getInstance().getRobotVelocity())),
                        this)
                .until(this::atAutoAlignGoal)
                .finallyDo(() -> {
                    isAutoAligning = false;
                    stop();
                });
    }

    public Command driveToPoseCommand(Supplier<Pose2d> goalPose) {
        return driveToPoseCommand(goalPose, RobotState.getInstance()::getEstimatedPose);
    }

    public Command feedforwardCharacterization() {
        return CharacterizationCommands.feedforward(
                        this::runFFCharacterization, this::getFFCharacterizationVelocity, this)
                .finallyDo(this::endFFCharacterization);
    }

    public Command wheelRadiusCharacterization() {
        return CharacterizationCommands.wheelRadius(
                input -> runVelocity(new ChassisSpeeds(0.0, 0.0, input)),
                () -> rawGyroRotation.getRadians(),
                this::getWheelRadiusCharacterizationPositions,
                this);
    }

    public static Drivetrain createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real drivetrain on simulated robot", false);
        return new Drivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
    }

    public static Drivetrain createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated drivetrain on real robot", false);

        SwerveDriveSimulation driveSim = SimState.getInstance().getDriveSim();
        return new Drivetrain(
                new GyroIOSim(driveSim.getGyroSimulation()),
                new ModuleIOSim(TunerConstants.FrontLeft, driveSim.getModules()[0]),
                new ModuleIOSim(TunerConstants.FrontRight, driveSim.getModules()[1]),
                new ModuleIOSim(TunerConstants.BackLeft, driveSim.getModules()[2]),
                new ModuleIOSim(TunerConstants.BackRight, driveSim.getModules()[3]));
    }

    public static Drivetrain createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy drivetrain on real robot", false);
        return new Drivetrain(
                new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
    }
}

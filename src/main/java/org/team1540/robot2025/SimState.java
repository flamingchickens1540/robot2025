package org.team1540.robot2025;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.generated.TunerConstants;
import org.team1540.robot2025.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2025.util.LoggedTracer;

public class SimState {
    private static SimState instance = null;

    public static SimState getInstance() {
        if (instance == null) instance = new SimState();
        return instance;
    }

    private final SwerveDriveSimulation driveSim;
    private final IntakeSimulation intakeSim;

    private SimState() {
        if (Constants.CURRENT_MODE != Constants.Mode.SIM)
            throw new IllegalStateException("SimState should only be used in simulation");

        SimulatedArena.getInstance().resetFieldForAuto();

        var simConfig = DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(Constants.ROBOT_MASS_KG))
                .withCustomModuleTranslations(DrivetrainConstants.getModuleTranslations())
                .withBumperSize(
                        Meters.of(Constants.BUMPER_LENGTH_X_METERS), Meters.of(Constants.BUMPER_LENGTH_Y_METERS))
                .withGyro(() -> new GyroSimulation(0.12 / 120, 0.02))
                .withSwerveModule(() -> new SwerveModuleSimulation(new SwerveModuleSimulationConfig(
                        DCMotor.getKrakenX60Foc(1),
                        DCMotor.getFalcon500(1),
                        TunerConstants.FrontLeft.DriveMotorGearRatio,
                        TunerConstants.FrontLeft.SteerMotorGearRatio,
                        Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                        Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                        Meters.of(TunerConstants.FrontLeft.WheelRadius),
                        KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                        DrivetrainConstants.WHEEL_COF)));
        driveSim = new SwerveDriveSimulation(simConfig, Pose2d.kZero);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "", driveSim, Inches.of(19.275), Inches.of(14.223), IntakeSimulation.IntakeSide.FRONT, 1);

        AutoLogOutputManager.addObject(this);
    }

    public void update() {
        LoggedTracer.reset();

        Logger.recordOutput(
                "SimState/Coral",
                SimulatedArena.getInstance().getGamePiecesByType("Coral").toArray(new Pose3d[0]));
        Logger.recordOutput(
                "SimState/Algae",
                SimulatedArena.getInstance().getGamePiecesByType("Algae").toArray(new Pose3d[0]));

        SimulatedArena.getInstance().simulationPeriodic();

        LoggedTracer.record("Simulation");
    }

    public SwerveDriveSimulation getDriveSim() {
        return driveSim;
    }

    @AutoLogOutput(key = "SimState/RobotPose")
    public Pose2d getSimulatedRobotPose() {
        return driveSim.getSimulatedDriveTrainPose();
    }

    public void resetSimPose(Pose2d pose) {
        driveSim.setSimulationWorldPose(pose);
    }

    public double[] getSimOdometryTimestamps() {
        double[] odometryTimestamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimestamps.length; i++) {
            odometryTimestamps[i] = Timer.getFPGATimestamp()
                    - 0.02
                    + i * SimulatedArena.getSimulationDt().in(Seconds);
        }
        return odometryTimestamps;
    }

    public void addIntakeData(Rotation2d pivotPosition) {
        if (pivotPosition.getDegrees() < 30) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }
    }
}

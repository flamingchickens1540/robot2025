package org.team1540.robot2025;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.generated.TunerConstants;
import org.team1540.robot2025.subsystems.drive.Drivetrain;

public class SimState {
    private static SimState instance = null;

    public static SimState getInstance() {
        if (instance == null) instance = new SimState();
        return instance;
    }

    private final SwerveDriveSimulation driveSim;

    private SimState() {
        if (Constants.kCurrentMode != Constants.Mode.SIM)
            throw new IllegalStateException("SimState should only be used in simulation");

        SimulatedArena.getInstance().resetFieldForAuto();

        var simConfig = DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(Constants.kRobotMassKg))
                .withCustomModuleTranslations(Drivetrain.kModuleTranslations)
                .withBumperSize(Meters.of(Constants.kBumperLengthXMeters), Meters.of(Constants.kBumperLengthYMeters))
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
                        Drivetrain.kWheelCOF)));
        driveSim = new SwerveDriveSimulation(simConfig, Pose2d.kZero);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
    }

    public void update() {
        Logger.recordOutput("SimState/RobotPose", getSimulatedRobotPose());
        Logger.recordOutput(
                "SimState/Coral",
                SimulatedArena.getInstance().getGamePiecesByType("Coral").toArray(new Pose3d[0]));
        Logger.recordOutput(
                "SimState/Algae",
                SimulatedArena.getInstance().getGamePiecesByType("Algae").toArray(new Pose3d[0]));

        SimulatedArena.getInstance().simulationPeriodic();
    }

    public SwerveDriveSimulation getDriveSim() {
        return driveSim;
    }

    public Pose2d getSimulatedRobotPose() {
        return driveSim.getSimulatedDriveTrainPose();
    }

    public void resetSimPose(Pose2d pose) {
        driveSim.setSimulationWorldPose(pose);
    }

    public double[] getSimulationOdometryTimestamps() {
        double[] odometryTimestamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimestamps.length; i++) {
            odometryTimestamps[i] = Timer.getFPGATimestamp()
                    - 0.02
                    + i * SimulatedArena.getSimulationDt().in(Seconds);
        }
        return odometryTimestamps;
    }
}

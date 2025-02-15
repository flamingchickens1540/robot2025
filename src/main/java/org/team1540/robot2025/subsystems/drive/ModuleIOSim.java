package org.team1540.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.team1540.robot2025.SimState;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two DC motor sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSim;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFF;
    private final PIDController turnPID;

    private Voltage driveAppliedVolts = Volts.zero();
    private Voltage turnAppliedVolts = Volts.zero();

    private boolean driveClosedLoop;
    private boolean turnClosedLoop;

    public ModuleIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
            SwerveModuleSimulation moduleSim) {
        this.moduleSim = moduleSim;
        this.driveMotor =
                moduleSim.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(constants.SlipCurrent));
        this.turnMotor = moduleSim.useGenericControllerForSteer().withCurrentLimit(Amps.of(40.0));

        drivePID = new PIDController(0.8, 0.0, 0.0);
        driveFF = new SimpleMotorFeedforward(0.05098, 0.83092);
        turnPID = new PIDController(75.0, 0.0, 0.0);

        turnPID.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVolts = Volts.of(
                    drivePID.calculate(moduleSim.getDriveWheelFinalSpeed().in(RotationsPerSecond))
                            + driveFF.calculate(drivePID.getSetpoint()));
        }
        if (turnClosedLoop) {
            turnAppliedVolts = Volts.of(
                    turnPID.calculate(moduleSim.getSteerAbsoluteFacing().getRotations()));
        }

        double batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
        driveAppliedVolts = Volts.of(MathUtil.clamp(driveAppliedVolts.in(Volts), -batteryVoltage, batteryVoltage));
        turnAppliedVolts = Volts.of(MathUtil.clamp(turnAppliedVolts.in(Volts), -batteryVoltage, batteryVoltage));

        driveMotor.requestVoltage(driveAppliedVolts);
        turnMotor.requestVoltage(turnAppliedVolts);

        inputs.driveConnected = true;
        inputs.drivePositionRads = moduleSim.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = driveAppliedVolts.in(Volts);
        inputs.driveSupplyCurrentAmps = moduleSim.getDriveMotorSupplyCurrent().in(Amps);
        inputs.driveStatorCurrentAmps = moduleSim.getDriveMotorStatorCurrent().in(Amps);

        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = moduleSim.getSteerAbsoluteFacing();
        inputs.turnPosition = inputs.turnAbsolutePosition;
        inputs.turnVelocityRadPerSec = moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVolts.in(Volts);
        inputs.turnSupplyCurrentAmps = moduleSim.getSteerMotorSupplyCurrent().in(Amps);
        inputs.turnStatorCurrentAmps = moduleSim.getSteerMotorStatorCurrent().in(Amps);

        inputs.odometryTimestamps = SimState.getInstance().getSimOdometryTimestamps();
        inputs.odometryDrivePositionsRads = Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();
        inputs.odometryTurnPositions = moduleSim.getCachedSteerAbsolutePositions();
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        drivePID.setSetpoint(Units.radiansToRotations(velocityRadPerSec));
    }

    @Override
    public void setDriveOpenLoop(double input) {
        driveClosedLoop = false;
        driveAppliedVolts = Volts.of(MathUtil.clamp(input, -12.0, 12.0));
        drivePID.reset();
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        turnClosedLoop = true;
        turnPID.setSetpoint(position.getRotations());
    }

    @Override
    public void setTurnOpenLoop(double input) {
        turnClosedLoop = false;
        turnAppliedVolts = Volts.of(MathUtil.clamp(input, -12.0, 12.0));
        turnPID.reset();
    }
}

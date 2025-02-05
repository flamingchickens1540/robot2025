package org.team1540.robot2025.subsystems.arm;

import static org.team1540.robot2025.subsystems.arm.ArmConstants.*;
// import static org.team1540.robot2025.subsystems.arm.ArmConstants.KP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.util.LoggedTunableNumber;
import org.team1540.robot2025.util.MechanismVisualizer;

public class Arm extends SubsystemBase {

    // fields:
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final LinearFilter positionFilter = LinearFilter.movingAverage(5); // units: rots
    private Rotation2d setpoint = new Rotation2d();
    private double avgPositionRots = 0;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", KD);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", KG);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kSF", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", KV);

    private static boolean hasInstance = false;

    public Arm(ArmIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of arm already exists");
        hasInstance = true;
        this.io = io;
    }

    public static Arm createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real shooter on simulated robot", false);
        }
        return new Arm(new ArmIOTalonFX());
    }

    public static Arm createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated arm on real robot", false);
        }
        return new Arm(new ArmIOSim());
    }

    public static Arm createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy arm on real robot", false);
        }
        return new Arm(new ArmIO() {});
    }

    public void periodic() {
        // update + process inputs!
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        MechanismVisualizer.setArmRotation(inputs.position);

        if (RobotState.isDisabled()) {
            io.setVoltage(0);
        }

        // update tunable numbers
        if (Constants.isTuningMode()
                && (kP.hasChanged(hashCode())
                        || kI.hasChanged(hashCode())
                        || kD.hasChanged(hashCode())
                        || kG.hasChanged(hashCode())
                        || kS.hasChanged(hashCode())
                        || kV.hasChanged(hashCode()))) {
            io.configPID(kP.get(), kI.get(), kD.get());
            io.configFeedForwardTerms(kG.get(), kS.get(), kV.get());
        }

        avgPositionRots = positionFilter.calculate(inputs.position.getRotations());
    }

    public void holdPosition() {
        setPosition(inputs.position);
    }

    public void setPosition(Rotation2d position) {
        setpoint = Rotation2d.fromRotations(
                MathUtil.clamp(position.getRotations(), MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations()));
        positionFilter.reset();
        io.setMotorPosition(setpoint);
    }

    public Rotation2d getPosition() {
        return inputs.position;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpoint.getRotations(), avgPositionRots, ERROR_TOLERANCE.getRotations());
    }

    public Command setPositionCommand(Supplier<Rotation2d> setpoint) {
        return new FunctionalCommand(
                () -> {}, () -> setPosition(setpoint.get()), (ignored) -> {}, this::isAtSetpoint, this);
    }

    @AutoLogOutput(key = "Arm/Setpoint")
    public Rotation2d getSetpoint() {
        return setpoint;
    }
}

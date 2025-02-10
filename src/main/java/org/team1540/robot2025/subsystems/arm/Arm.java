package org.team1540.robot2025.subsystems.arm;

import static org.team1540.robot2025.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.MechanismVisualizer;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    // fields:
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private Rotation2d setpoint = new Rotation2d();

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", KD);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", KG);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", KV);

    private static boolean hasInstance = false;

    private Arm(ArmIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of arm already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {
        // update + process inputs!
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        MechanismVisualizer.getInstance().setArmRotation(inputs.position);

        if (RobotState.isDisabled()) {
            io.setVoltage(0);
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.configFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);
    }

    public void holdPosition() {
        setPosition(inputs.position);
    }

    public void setPosition(Rotation2d position) {
        setpoint = Rotation2d.fromRotations(
                MathUtil.clamp(position.getRotations(), MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations()));
        io.setSetpoint(setpoint);
    }

    public Rotation2d getPosition() {
        return inputs.position;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    @AutoLogOutput(key = "Arm/AtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpoint.getRotations(), inputs.position.getRotations(), ERROR_TOLERANCE.getRotations());
    }

    public Command setpointCommand(ArmState state) {
        return Commands.run(() -> setPosition(state.angle), this).until(this::isAtSetpoint);
    }

    @AutoLogOutput(key = "Arm/Setpoint")
    public Rotation2d getSetpoint() {
        return setpoint;
    }

    public static Arm createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real arm on simulated robot", false);
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
}

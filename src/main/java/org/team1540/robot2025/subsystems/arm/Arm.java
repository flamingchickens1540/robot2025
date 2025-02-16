package org.team1540.robot2025.subsystems.arm;

import static org.team1540.robot2025.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.Constants;
import org.team1540.robot2025.services.MechanismVisualizer;
import org.team1540.robot2025.subsystems.elevator.Elevator;
import org.team1540.robot2025.subsystems.intake.CoralIntake;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    private static boolean hasInstance = false;

    public enum ArmState {
        STOW(new LoggedTunableNumber("Arm/Setpoints/StowDegrees", 120)),
        INTAKE(new LoggedTunableNumber("Arm/Setpoints/IntakeDegrees", 60)),
        REEF_ALGAE(new LoggedTunableNumber("Arm/Setpoints/ReefAlgaeDegrees", 0)),
        SCORE_L2_L3(new LoggedTunableNumber(
                "Arm/Setpoints/ScoreL2L3Degrees", 125)), // TODO: talk to Simon abt getting happy values
        SCORE_L1(new LoggedTunableNumber("Arm/Setpoints/ScoreL1Degrees", 140)),
        SCORE_L4(new LoggedTunableNumber("Arm/Setpoints/ScoreL4Degrees", 180)),
        FUNNEL(new LoggedTunableNumber("Arm/Setpoints/FunnelDegrees", 115)),
        GROUND_ALGAE(new LoggedTunableNumber("Arm/Setpoints/GroundAlgaeDegrees", 210)),
        BARGE(new LoggedTunableNumber("Arm/Setpoints/BargeDegrees", 60)),
        SCORE_L1_REVERSE(new LoggedTunableNumber("Arm/Setpoints/ScoreL1ReverseDegrees", 40)),
        SCORE_L2_L3_REVERSE(new LoggedTunableNumber("Arm/Setpoints/ScoreL2L3ReverseDegrees", 55)),
        SCORE_L4_REVERSE(new LoggedTunableNumber("Arm/Setpoints/ScoreL4ReverseDegrees", 0)),
        REEF_ALGAE_REVERSE(new LoggedTunableNumber("Arm/Setpoints/ReefAlgaeReverseDegrees", 0)),
        BARGE_REVERSE(new LoggedTunableNumber("Arm/Setpoints/BargeReverseDegrees", 0)),
        PROCESSOR(new LoggedTunableNumber("Arm/Setpoints/ProcessorDegrees", 0));
        // TODO: stow with algae, back for most,
        private final DoubleSupplier positionDegrees;

        ArmState(DoubleSupplier positionDegrees) {
            this.positionDegrees = positionDegrees;
        }

        public Rotation2d position() {
            return Rotation2d.fromDegrees(positionDegrees.getAsDouble());
        }
    }

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private Rotation2d setpoint = new Rotation2d();

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", KD);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", KG);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", KV);

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

    @AutoLogOutput(key = "Arm/Setpoint")
    public Rotation2d getSetpoint() {
        return setpoint;
    }

    public Command commandToSetpoint(ArmState state) {
        return Commands.run(() -> setPosition(state.position()), this).until(this::isAtSetpoint);
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

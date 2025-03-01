package org.team1540.robot2025.subsystems.arm;

import static org.team1540.robot2025.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import org.team1540.robot2025.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    private static boolean hasInstance = false;

    public enum ArmState {
        STOW(new LoggedTunableNumber("Arm/Setpoints/StowDegrees", 120)),
        STOW_ALGAE(new LoggedTunableNumber("Arm/Setpoints/StowAlgaeDegrees", 136)),

        INTAKE(new LoggedTunableNumber("Arm/Setpoints/IntakeDegrees", 49.5)),
        FUNNEL(new LoggedTunableNumber("Arm/Setpoints/FunnelDegrees", 100)),

        REEF_ALGAE_FRONT(new LoggedTunableNumber("Arm/Setpoints/ReefAlgaeFrontDegrees", 0)), // TODO: get value
        REEF_ALGAE_BACK(new LoggedTunableNumber("Arm/Setpoints/ReefAlgaeBackDegrees", 180)),
        GROUND_ALGAE(new LoggedTunableNumber("Arm/Setpoints/GroundAlgaeDegrees", 220)),
        PROCESSOR(new LoggedTunableNumber("Arm/Setpoints/Processor", 177)),
        SCORE_L1_BACK(new LoggedTunableNumber("Arm/Setpoints/ScoreL1BackDegrees", 250)),
        SCORE_L1_FRONT(new LoggedTunableNumber("Arm/Setpoints/ScoreL1FrontDegrees", 250)),
        SCORE_L2_L3_FRONT(new LoggedTunableNumber("Arm/Setpoints/ScoreL2L3FrontDegrees", 55)),
        SCORE_L2_L3_BACK(new LoggedTunableNumber("Arm/Setpoints/ScoreL2L3BackDegrees", 115)),
        SCORE_L4_FRONT(new LoggedTunableNumber("Arm/Setpoints/ScoreL4FrontDegrees", 60)),
        SCORE_L4_BACK(new LoggedTunableNumber("Arm/Setpoints/ScoreL4BackDegrees", 118)),
        SCORE_BARGE_FRONT(new LoggedTunableNumber("Arm/Setpoints/ScoreBargeFrontDegrees", 90)),
        SCORE_BARGE_BACK(new LoggedTunableNumber("Arm/Setpoints/ScoreBargeBackDegrees", 0));

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

    private final TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(CRUISE_VELOCITY_RPS, MAX_ACCEL_RPS2));

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

    @AutoLogOutput(key = "Arm/TimeToSetpoint")
    public double timeToSetpoint() {
        return timeToSetpoint(getSetpoint());
    }

    public double timeToSetpoint(Rotation2d setpoint) {
        trapezoidProfile.calculate(
                0.0,
                new TrapezoidProfile.State(getPosition().getRotations(), inputs.velocityRPM),
                new TrapezoidProfile.State(setpoint.getRotations(), 0));
        return trapezoidProfile.totalTime();
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

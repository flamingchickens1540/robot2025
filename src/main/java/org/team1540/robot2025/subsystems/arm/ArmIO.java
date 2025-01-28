package org.team1540.robot2025.subsystems.arm;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
public interface ArmIO {
    @AutoLog
    class ArmIOInputs{
        //TODO: are these inputs okay? do I want any other ones?
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public Rotation2d rotation2d = new Rotation2d();
        public double tempCelsius = 0.0;
    }

    default void setVoltage(double voltage){}
    default void updateInputs(ArmIOInputs inputs){}
    default void setSetPoint(Rotation2d setPoint){};
    default void configPID(double kP, double kI, double kD){};
    default void setBrakeMode(boolean setBrake){};
}

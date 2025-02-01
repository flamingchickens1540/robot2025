package org.team1540.robot2025.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public Rotation2d position = new Rotation2d();
        public double tempCelsius = 0.0;
        public boolean isAtForwardLimit = false;
        public boolean isAtReverseLimit = false;
    }

    // runs open loop at given voltage
    default void setVoltage(double voltage) {}

    // updates the loggable inputs
    default void updateInputs(ArmIOInputs inputs) {}

    // runs closed loop to given position
    default void setPosition(Rotation2d position) {}

    // configures the PID controller
    default void configPID(double kP, double kI, double kD, double kG) {}

    // sets neutral output mode, either coast or brake mode
    default void setBrakeMode(boolean setBrake) {}

    default void setEncoderPosition(double rots) {}
}

package org.team1540.robot2025.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public boolean motorConnected = false;

        public Rotation2d position = new Rotation2d();
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public boolean isAtForwardLimit = false;
        public boolean isAtReverseLimit = false;
    }

    // runs open loop at given voltage
    default void setVoltage(double voltage) {}

    // updates the loggable inputs
    default void updateInputs(ClimberIOInputs inputs) {}

    // runs closed loop to given position
    default void setSetpoint(Rotation2d motorPosition) {}

    default void resetPivotPosition(Rotation2d position) {}

    // configures the PID controller
    default void configPID(double kP, double kI, double kD) {}

    // updates feedforward terms
    default void configFF(double kS, double kV, double kG) {}

    // sets neutral output mode, either coast or brake mode
    default void setBrakeMode(boolean setBrake) {}
}

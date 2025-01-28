package org.team1540.robot2025.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
    @AutoLog
    class GrabberIOInputs {
        public double motorCurrentAmps = 0.0;
        public double motorAppliedVolts = 0.0;
        public double motorTempCelsius = 0.0;
        public double velocityRPM = 0.0;
        public double positionRots = 0.0;
        public boolean hasCoral = false;
    }

    default void updateInputs(GrabberIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setVelocity(double velocityRPM) {}

    default void setBrakeMode(boolean isBrakeMode) {}
}

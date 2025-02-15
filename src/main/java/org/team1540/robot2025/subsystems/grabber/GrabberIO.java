package org.team1540.robot2025.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
    @AutoLog
    class GrabberIOInputs {
        public boolean motorConnected = false;

        public double motorSupplyCurrentAmps = 0.0;
        public double motorStatorCurrentAmps = 0.0;
        public double motorAppliedVolts = 0.0;
        public double motorTempCelsius = 0.0;
        public double motorVelocityRPM = 0.0;
    }

    default void updateInputs(GrabberIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setBrakeMode(boolean isBrakeMode) {}
}

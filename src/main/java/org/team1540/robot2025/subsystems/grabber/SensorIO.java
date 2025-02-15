package org.team1540.robot2025.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
    @AutoLog
    class SensorIOInputs {
        public boolean beforeSensorConnected = true;
        public boolean afterSensorConnected = true;
        public boolean beforeSensorTripped = false;
        public boolean afterSensorTripped = false;
    }

    default void updateInputs(SensorIOInputs inputs) {}
}

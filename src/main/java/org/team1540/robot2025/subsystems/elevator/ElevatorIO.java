package org.team1540.robot2025.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs {


        public double leaderCurrentAmps = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double leaderTempCelsius = 0.0;
        public double leaderPositionMeters = 0.0;
        public double leaderVelocityMPS = 0.0;

        public double followerCurrentAmps = 0.0;
        public double followerAppliedVolts = 0.0;
        public double followerTempCelsius = 0.0;
        public double followerPositionMeters = 0.0;
        public double followerVelocityMPS = 0.0;


        public boolean atUpperLimit = false;
        public boolean atLowerLimit = false;

    }

    default void updateInputs(ElevatorIOInputs inputs) {}
    default void setVoltage(double volts) {}
    default void setSetpointMeters(double setpoint) {}
    default void setSetpointAngle(Angle setpoint) {}
    default void setBreakMode(boolean breakMode) {}
}

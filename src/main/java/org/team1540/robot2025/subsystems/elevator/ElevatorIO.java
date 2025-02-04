package org.team1540.robot2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs {

        public double[] supplyCurrentAmps = new double[2];
        public double[] statorCurrentAmps = new double[2];
        public double[] appliedVolts = new double[2];
        public double[] tempCelsius = new double[2];
        public double[] positionMeters = new double[2];
        public double[] velocityMPS = new double[2];
        public double[] connection;

        public boolean atUpperLimit = false;
        public boolean atLowerLimit = false;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setSetpoint(double setpointMeters) {}

    default void setBrakeMode(boolean brakeMode) {}

    default void configPID(double kP, double kI, double kD) {}

    default void configFF(double kS, double kV, double kA) {}
}

package org.team1540.robot2025.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public boolean motorConnected = false;

        public Rotation2d pivotPosition = new Rotation2d();
        public double pivotVelocityRPM = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotSupplyCurrentAmps = 0.0;
        public double pivotStatorCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;

        public boolean isAtForwardLimit = false;
        public boolean isAtReverseLimit = false;

        public boolean rollerConnected = true;
        public double rollerVelocityRPM = 0;
        public double rollerAppliedVolts = 0;
        public double rollerSupplyCurrentAmps = 0;
        public double rollerStatorCurrentAmps = 0;
    }

    // runs open loop at given voltage
    default void setPivotVoltage(double voltage) {}

    default void setRollerVoltage(double voltage) {}

    // updates the loggable inputs
    default void updateInputs(ClimberIOInputs inputs) {}

    // runs closed loop to given position
    default void setPivotSetpoint(Rotation2d motorPosition) {}

    default void resetPivotPosition(Rotation2d position) {}

    // configures the PID controller
    default void configPID(double kP, double kI, double kD) {}

    // updates feedforward terms
    default void configFF(double kS, double kV, double kG) {}

    // sets neutral output mode, either coast or brake mode
    default void setBrakeMode(boolean setBrake) {}
}

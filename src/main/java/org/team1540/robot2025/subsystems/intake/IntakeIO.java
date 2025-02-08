package org.team1540.robot2025.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class CoralIntakeInputs {
        public boolean spinConnected = true;
        public double spinMotorPosition = 0;
        public double spinMotorVelocityRPS = 0;
        public double spinMotorAppliedVolts = 0;
        public double spinSupplyCurrent = 0;
        public double spinStatorCurrent = 0;

        public boolean funnelConnected = true;
        public double funnelMotorPosition = 0;
        public double funnelMotorVelocityRPS = 0;
        public double funnelAppliedVolts = 0;
        public double funnelMotorAppliedVolts = 0;
        public double funnelSupplyCurrent = 0;
        public double funnelStatorCurrent = 0;

        public boolean pivotConnected = true;
        public double pivotMotorPosition = 0;
        public double pivotMotorVelocityRPS = 0;
        public double pivotMotorAppliedVolts = 0;
        public double pivotSupplyCurrent = 0;
        public double pivotStatorCurrent = 0;
    }

    default void setRollerVoltage(double speed) {}

    default void setFunnelVoltage(double speed) {}

    default void setPivotSetpoint(Rotation2d rotations) {}

    default void setPivotVoltage(double voltage) {}

    default void setPivotPID(double kP, double kI, double kD) {}

    default void setPivotFF(double kS, double kV, double kG) {}

    default void updateInputs(CoralIntakeInputs inputs) {}
}

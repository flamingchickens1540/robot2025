package org.team1540.robot2025.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        public boolean spinConnected = true;
        public double spinMotorVelocityRPS = 0;
        public double spinMotorAppliedVolts = 0;
        public double spinSupplyCurrentAmps = 0;
        public double spinStatorCurrentAmps = 0;

        public boolean funnelConnected = true;
        public double funnelMotorVelocityRPS = 0;
        public double funnelMotorAppliedVolts = 0;
        public double funnelSupplyCurrentAmps = 0;
        public double funnelStatorCurrentAmps = 0;

        public boolean pivotConnected = true;
        public Rotation2d pivotPosition = Rotation2d.kZero;
        public double pivotMotorVelocityRPS = 0;
        public double pivotMotorAppliedVolts = 0;
        public double pivotSupplyCurrentAmps = 0;
        public double pivotStatorCurrentAmps = 0;
    }

    default void setRollerVoltage(double voltage) {}

    default void setFunnelVoltage(double voltage) {}

    default void setPivotSetpoint(Rotation2d rotations) {}

    default void resetPivotPosition(Rotation2d rotations) {}

    default void setPivotVoltage(double voltage) {}

    default void setPivotPID(double kP, double kI, double kD) {}

    default void setPivotFF(double kS, double kV, double kG) {}

    default void updateInputs(IntakeInputs inputs) {}
}

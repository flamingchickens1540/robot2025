package org.team1540.robot2025.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    class CoralIntakeInputs {

        public double spinMotorPosition = 0;
        public double spinMotorVelocityRPS = 0;
        public double spinMotorAppliedVolts = 0;
        public double spinCurrentAmps = 0;

        public double pivotMotorPosition = 0;
        public double pivotMotorVelocityRPS = 0;
        public double pivotMotorAppliedVolts = 0;
        public double pivotCurrentAmps = 0;

        public double neoMotorAppliedOutput = 0;
    }

    default void setRollerSpeed(double speed) {}

    default void setNEOSpeed(double speed) {}

    default void setPivot(Rotation2d rotations) {}

    default void updateInputs(CoralIntakeInputs inputs) {}
}

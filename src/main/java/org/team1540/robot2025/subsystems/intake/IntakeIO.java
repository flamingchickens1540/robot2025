package org.team1540.robot2025.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    class CoralIntakeInputs {

        public double topMotorPosition = 0;
        public double topMotorVelocityRPS = 0;
        public double topMotorAppliedVolts = 0;
        public double topCurrentAmps = 0;

        public double bottomMotorPosition = 0;
        public double bottomMotorVelocityRPS = 0;
        public double bottomMotorAppliedVolts = 0;
        public double bottomCurrentAmps = 0;

        public double neoMotorAppliedOutput = 0;
    }

    default void setSpeed(double speed) {}

    default void setPosition(Rotation2d rotations) {}

    default void updateInputs(CoralIntakeInputs inputs) {}
}

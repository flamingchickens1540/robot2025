package org.team1540.robot2025.services;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.subsystems.arm.ArmConstants;
import org.team1540.robot2025.subsystems.elevator.ElevatorConstants;
import org.team1540.robot2025.subsystems.intake.CoralIntakeConstants;

public class MechanismVisualizer {
    private static MechanismVisualizer instance = null;

    public static MechanismVisualizer getInstance() {
        if (instance == null) instance = new MechanismVisualizer();
        return instance;
    }

    private Rotation2d armPosition = Rotation2d.kZero;
    private Rotation2d intakePosition = Rotation2d.kZero;
    private double elevatorPositionMeters = 0.0;

    public void update() {
        Pose3d elevatorCarriage = new Pose3d(0.0, 0.0, elevatorPositionMeters, Rotation3d.kZero);
        Pose3d elevatorStage1 = new Pose3d(
                0.0, 0.0, Math.max(0.0, elevatorPositionMeters - ElevatorConstants.STAGE_1_HEIGHT_M), Rotation3d.kZero);

        Pose3d arm = new Pose3d(
                ArmConstants.ROTATIONAL_ORIGIN.plus(new Translation3d(0.0, 0.0, elevatorPositionMeters)),
                new Rotation3d(0.0, -armPosition.getRadians(), 0.0));

        Pose3d intake = new Pose3d(
                CoralIntakeConstants.ROTATIONAL_ORIGIN,
                new Rotation3d(0.0, Math.toRadians(90) - intakePosition.getRadians(), 0.0));

        Logger.recordOutput("Mechanisms", elevatorCarriage, elevatorStage1, arm, intake);
    }

    public void setElevatorPosition(double positionMeters) {
        elevatorPositionMeters = positionMeters;
    }

    public void setArmRotation(Rotation2d rotation) {
        armPosition = rotation;
    }

    public void setIntakeRotation(Rotation2d rotation) {
        intakePosition = rotation;
    }
}

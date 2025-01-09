package org.team1540.robot2025;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.subsystems.drive.Drivetrain;

public class RobotState {
    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private final SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private final Timer resetTimer = new Timer();

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    //    private Trajectory<SwerveSample> activeTrajectory = null;
    private Pose2d[] activeTrajectory;

    private final Field2d field = new Field2d();

    private RobotState() {
        poseEstimator = new SwerveDrivePoseEstimator(
                Drivetrain.kKinematics,
                lastGyroRotation,
                lastModulePositions,
                Pose2d.kZero,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, 5.0));
        resetTimer.start();

        SmartDashboard.putData(field);
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle, double timestamp) {
        lastModulePositions = modulePositions;
        lastGyroRotation = gyroAngle;
        poseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions);
        field.setRobotPose(getEstimatedPose());
    }

    public void addVelocityData(ChassisSpeeds velocity) {
        robotVelocity = velocity;
    }

    public void setActiveTrajectory(Pose2d... poses) {
        activeTrajectory = poses;
        field.getObject("trajectory").setPoses(activeTrajectory);
        Logger.recordOutput("Odometry/Trajectory/ActiveTrajectory", activeTrajectory);
    }

    public void clearActiveTrajectory() {
        field.getObject("trajectory").setPoses();
        Logger.recordOutput("Odometry/Trajectory/ActiveTrajectory", new Pose2d[0]);
        activeTrajectory = null;
    }

    public void setTrajectoryTarget(Pose2d target) {
        Logger.recordOutput("Odometry/Trajectory/TargetPose", target);
    }

    public void resetPose(Pose2d newPose) {
        if (Constants.kCurrentMode == Constants.Mode.SIM) SimState.getInstance().resetSimPose(newPose);
        poseEstimator.resetPosition(lastGyroRotation, lastModulePositions, newPose);
        resetTimer.restart();
    }

    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRobotRotation() {
        return getEstimatedPose().getRotation();
    }

    @AutoLogOutput(key = "Odometry/RobotVelocity")
    public ChassisSpeeds getRobotVelocity() {
        return robotVelocity;
    }

    @AutoLogOutput(key = "Odometry/FieldRelativeVelocity")
    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getRobotRotation());
    }

    public Pose2d predictRobotPose(double lookaheadSeconds) {
        ChassisSpeeds velocity = getFieldRelativeVelocity();
        Pose2d pose = getEstimatedPose();
        return new Pose2d(
                pose.getX() + velocity.vxMetersPerSecond * lookaheadSeconds,
                pose.getY() + velocity.vyMetersPerSecond * lookaheadSeconds,
                pose.getRotation().plus(Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * lookaheadSeconds)));
    }
}

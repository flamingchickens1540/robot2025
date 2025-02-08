package org.team1540.robot2025;

import static org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVisionIO;

public class RobotState {
    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(DrivetrainConstants.getModuleTranslations());
    private final SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private final Timer resetTimer = new Timer();

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private Pose2d[] activeTrajectory;

    private final Field2d field = new Field2d();

    private RobotState() {
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                lastGyroRotation,
                lastModulePositions,
                Pose2d.kZero,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, 5.0));
        resetTimer.start();

        SmartDashboard.putData(field);

        AutoLogOutputManager.addObject(this);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle, double timestamp) {
        lastModulePositions = modulePositions;
        lastGyroRotation = gyroAngle;
        poseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions);
        field.setRobotPose(getEstimatedPose());
    }

    public void addVisionMeasurement(AprilTagVisionIO.PoseObservation visionPose) {
        if (shouldAcceptVision(visionPose)) {
            poseEstimator.addVisionMeasurement(
                    visionPose.estimatedPoseMeters().toPose2d(),
                    visionPose.lastMeasurementTimestampSecs(),
                    getStdDevs(visionPose));
        }
    }

    private Matrix<N3, N1> getStdDevs(AprilTagVisionIO.PoseObservation poseObservation) {
        double xyStdDev =
                XY_STD_DEV_COEFF * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        double rotStdDev =
                ROT_STD_DEV_COEFF * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        boolean acceptYaw =
                poseObservation.numTagsSeen() > 1 || (poseObservation.numTagsSeen() > 0 && DriverStation.isDisabled());
        return VecBuilder.fill(xyStdDev, xyStdDev, acceptYaw ? rotStdDev : Double.POSITIVE_INFINITY);
    }

    private boolean shouldAcceptVision(AprilTagVisionIO.PoseObservation poseObservation) {
        Pose3d estimatedPose = poseObservation.estimatedPoseMeters();
        return poseObservation.numTagsSeen() >= MIN_ACCEPTED_NUM_TAGS // Must see sufficient tags
                // Must be within field roughly
                && estimatedPose.getX() >= -MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getX() <= APRIL_TAG_FIELD_LAYOUT.getFieldLength() + MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getY() >= -MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getY() <= APRIL_TAG_FIELD_LAYOUT.getFieldWidth() + MAX_OUTSIDE_OF_FIELD_TOLERANCE
                // Must not be actively flying
                && estimatedPose.getZ() <= MAX_ROBOT_Z_TOLERANCE;
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
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) SimState.getInstance().resetSimPose(newPose);
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

package org.team1540.robot2025;

import static org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2025.FieldConstants.ReefFace;
import org.team1540.robot2025.subsystems.drive.DrivetrainConstants;
import org.team1540.robot2025.subsystems.vision.apriltag.AprilTagVisionIO;
import org.team1540.robot2025.util.AllianceFlipUtil;
import org.team1540.robot2025.util.LoggedTunableNumber;

public class RobotState {
    private static final LoggedTunableNumber singleTagObservationStaleSecs =
            new LoggedTunableNumber("Odometry/SingleTagObservationStaleSecs", 0.5);
    private static final LoggedTunableNumber tagPoseMinBlendDistanceMeters =
            new LoggedTunableNumber("Odometry/TagPoseMinBlendDistanceMeters", Units.inchesToMeters(24.0));
    private static final LoggedTunableNumber tagPoseMaxBlendDistanceMeters =
            new LoggedTunableNumber("Odometry/TagPoseMaxBlendDistanceMeters", Units.inchesToMeters(36.0));

    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(DrivetrainConstants.getModuleTranslations());
    private final SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private Pose2d odometryPose = Pose2d.kZero;
    private final TimeInterpolatableBuffer<Pose2d> odometryPoseBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
    private final TimeInterpolatableBuffer<Pose2d> fusedPoseBuffer = TimeInterpolatableBuffer.createBuffer(2.0);

    private final Timer resetTimer = new Timer();

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private final SingleTagPoseEstimate[] singleTagPoses = new SingleTagPoseEstimate[FieldConstants.aprilTagCount];

    private Pose2d[] activeTrajectory;

    private final Field2d field = new Field2d();

    private RobotState() {
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                lastGyroRotation,
                lastModulePositions,
                Pose2d.kZero,
                VecBuilder.fill(0.05, 0.05, 0.1),
                VecBuilder.fill(0.5, 0.5, 5.0));
        resetTimer.start();

        SmartDashboard.putData(field);

        AutoLogOutputManager.addObject(this);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle, double timestamp) {
        Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
        lastModulePositions = modulePositions;
        lastGyroRotation = gyroAngle;

        odometryPose = odometryPose.exp(twist);
        odometryPose = new Pose2d(odometryPose.getTranslation(), gyroAngle);
        odometryPoseBuffer.addSample(timestamp, odometryPose);

        poseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions);
        fusedPoseBuffer.addSample(timestamp, poseEstimator.getEstimatedPosition());
        field.setRobotPose(getEstimatedPose());
    }

    public boolean addVisionMeasurement(AprilTagVisionIO.PoseObservation visionPose) {
        if (shouldAcceptVision(visionPose) && resetTimer.hasElapsed(0.1)) {
            poseEstimator.addVisionMeasurement(
                    visionPose.estimatedPoseMeters().toPose2d(), visionPose.timestampSecs(), getStdDevs(visionPose));
            fusedPoseBuffer.addSample(visionPose.timestampSecs(), poseEstimator.getEstimatedPosition());
            return true;
        }
        return false;
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
                && estimatedPose.getX() <= FieldConstants.fieldLength + MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getY() >= -MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getY() <= FieldConstants.fieldWidth + MAX_OUTSIDE_OF_FIELD_TOLERANCE
                // Must not be actively flying
                && Math.abs(estimatedPose.getZ()) <= MAX_ROBOT_Z_TOLERANCE;
    }

    public void addSingleTagMeasurement(AprilTagVisionIO.SingleTagObservation observation) {
        int tagIndex = observation.id() - 1;

        // Skip if measurement is not the most recent
        if (singleTagPoses[tagIndex] != null && singleTagPoses[tagIndex].timestamp() >= observation.timestampSecs()) {
            return;
        }

        Optional<Pose2d> poseBufferSample = odometryPoseBuffer.getSample(observation.timestampSecs());
        if (poseBufferSample.isEmpty()) {
            return;
        }
        Rotation2d robotRotation = poseBufferSample.get().getRotation();

        Pose3d robotToCameraPose = Pose3d.kZero.transformBy(observation.cameraTransform());

        Translation2d cameraToTagTranslation = new Pose3d(
                        Translation3d.kZero,
                        new Rotation3d(
                                0.0,
                                -observation.pitch().getRadians(),
                                -observation.yaw().getRadians()))
                .transformBy(new Transform3d(observation.distanceMeters(), 0, 0, Rotation3d.kZero))
                .getTranslation()
                .rotateBy(new Rotation3d(
                        0, observation.cameraTransform().getRotation().getY(), 0))
                .toTranslation2d();
        Rotation2d cameraToTagRotation =
                robotRotation.plus(robotToCameraPose.toPose2d().getRotation().plus(cameraToTagTranslation.getAngle()));
        Optional<Pose3d> tagPose = FieldConstants.aprilTagLayout.getTagPose(observation.id());
        if (tagPose.isEmpty()) return;
        Translation2d fieldToCameraTranslation = new Pose2d(
                        tagPose.get().toPose2d().getTranslation(), cameraToTagRotation.plus(Rotation2d.k180deg))
                .transformBy(new Transform2d(cameraToTagTranslation.getNorm(), 0.0, Rotation2d.kZero))
                .getTranslation();
        Pose2d robotPose = new Pose2d(
                        fieldToCameraTranslation,
                        robotRotation.plus(robotToCameraPose.toPose2d().getRotation()))
                .transformBy(new Transform2d(robotToCameraPose.toPose2d(), Pose2d.kZero));
        robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

        singleTagPoses[tagIndex] =
                new SingleTagPoseEstimate(robotPose, observation.distanceMeters(), observation.timestampSecs());
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
        odometryPose = newPose;
        odometryPoseBuffer.clear();
        fusedPoseBuffer.clear();
        resetTimer.restart();
    }

    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRobotRotation() {
        return getEstimatedPose().getRotation();
    }

    public Optional<Pose2d> getSingleTagPose(int tagID) {
        int tagIndex = tagID - 1;
        if (singleTagPoses[tagIndex] == null) {
            return Optional.empty();
        }
        // Check if observation is stale
        SingleTagPoseEstimate poseEstimate = singleTagPoses[tagIndex];
        if (Timer.getTimestamp() - poseEstimate.timestamp() >= singleTagObservationStaleSecs.get()) {
            return Optional.empty();
        }

        Optional<Pose2d> odometryPoseAtTime = odometryPoseBuffer.getSample(poseEstimate.timestamp());
        return odometryPoseAtTime.map(pose -> poseEstimate.pose().plus(new Transform2d(pose, odometryPose)));
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

    public Pose2d getReefAlignmentPose(ReefFace face) {
        boolean isRed = AllianceFlipUtil.shouldFlip();
        int tagID = isRed ? face.redTagID() : face.blueTagID();
        Optional<Pose2d> singleTagPose = getSingleTagPose(tagID);

        // If no single tag pose, return global pose estimate
        if (singleTagPose.isEmpty()) return getEstimatedPose();

        Pose2d tagPose = FieldConstants.aprilTagLayout
                .getTagPose(tagID)
                .orElse(Pose3d.kZero)
                .toPose2d();
        double t = MathUtil.clamp(
                (getEstimatedPose().getTranslation().getDistance(tagPose.getTranslation())
                                - tagPoseMinBlendDistanceMeters.get())
                        / (tagPoseMaxBlendDistanceMeters.get() - tagPoseMinBlendDistanceMeters.get()),
                0.0,
                1.0);
        return getEstimatedPose().interpolate(singleTagPose.get(), 1.0 - t);
    }

    public boolean shouldReverseCoral() {
        return Math.abs(AllianceFlipUtil.maybeFlipPose(
                                FieldConstants.Reef.closestFace().get().pose())
                        .getRotation()
                        .minus(RobotState.getInstance().getRobotRotation())
                        .getDegrees())
                > 90;
    }

    public boolean shouldReverseCoral(FieldConstants.ReefBranch branch) {
        return Math.abs(AllianceFlipUtil.maybeFlipPose(branch.face.pose())
                        .getRotation()
                        .minus(RobotState.getInstance().getRobotRotation())
                        .getDegrees())
                > 90;
    }

    public void periodicLog() {
        for (int i = 0; i < singleTagPoses.length; i++) {
            Optional<Pose2d> singleTagEstimate = getSingleTagPose(i + 1);
            if (singleTagEstimate.isPresent()) {
                Logger.recordOutput("Odometry/SingleTagPoses/" + (i + 1), singleTagEstimate.get());
            }
        }
    }

    public record SingleTagPoseEstimate(Pose2d pose, double distance, double timestamp) {}
}

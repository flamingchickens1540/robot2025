package org.team1540.robot2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.*;
import java.util.function.Supplier;
import org.team1540.robot2025.subsystems.grabber.GrabberConstants;
import org.team1540.robot2025.util.AllianceFlipUtil;

// NOTE this file is available at:
// https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/FieldConstants.java

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    public static final double startingLineX =
            Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double algaeDiameter = Units.inchesToMeters(16);

    public static class Processor {
        public static final Pose2d centerFace =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d leftCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d rightCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace = new Pose2d(
                Units.inchesToMeters(33.526), Units.inchesToMeters(291.176), Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace = new Pose2d(
                Units.inchesToMeters(33.526), Units.inchesToMeters(25.824), Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
        public static final double faceLength = Units.inchesToMeters(36.792600);
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
        public static final double faceToZoneLine =
                Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        private static final Pose2d[] centerFaces = new Pose2d[] {
            new Pose2d(Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180)),
            new Pose2d(Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120)),
            new Pose2d(Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60)),
            new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60)),
            new Pose2d(Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
        }; // Starting facing the driver station in counterclockwise order

        public static final List<Map<ReefHeight, Pose3d>> branchPositions =
                new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
        public static final List<Pose2d> scorePositions = new ArrayList<>();
        public static final List<ReefFace> faces =
                new ArrayList<>(); // Starting facing the driver station in counterclockwise order

        static {
            // Initialize branch positions
            for (int face = 0; face < 6; face++) {
                Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
                Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
                for (var level : ReefHeight.values()) {
                    Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                    double adjustX = Units.inchesToMeters(30.738);
                    double adjustY = Units.inchesToMeters(6.469);

                    fillRight.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                    fillLeft.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                }
                branchPositions.add(fillLeft);
                branchPositions.add(fillRight);

                scorePositions.add(centerFaces[face].transformBy(new Transform2d(
                        Constants.BUMPER_LENGTH_X_METERS / 2,
                        GrabberConstants.Y_OFFSET_METERS - Units.inchesToMeters(6.469),
                        Rotation2d.k180deg)));
                scorePositions.add(centerFaces[face].transformBy(new Transform2d(
                        Constants.BUMPER_LENGTH_X_METERS / 2,
                        GrabberConstants.Y_OFFSET_METERS + Units.inchesToMeters(6.469),
                        Rotation2d.k180deg)));

                faces.add(new ReefFace(
                        centerFaces[face],
                        scorePositions.get(scorePositions.size() - 2),
                        scorePositions.get(scorePositions.size() - 1)));
            }
        }

        public static Supplier<Pose2d> closestBranch() {
            return () -> {
                Pose2d closestBranch = new Pose2d();
                double closestDistance = Double.MAX_VALUE;

                for (Pose2d pose : FieldConstants.Reef.scorePositions) {
                    pose = AllianceFlipUtil.maybeFlipPose(pose);
                    double distance = RobotState.getInstance()
                            .getEstimatedPose()
                            .minus(pose)
                            .getTranslation()
                            .getNorm();
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestBranch = pose;
                    }
                }
                return closestBranch;
            };
        }

        public static Supplier<Pose2d> closestFace() {
            return () -> {
                Pose2d closestFace = new Pose2d();
                double closestDistance = Double.MAX_VALUE;

                for (Pose2d pose : Reef.centerFaces) {
                    pose = AllianceFlipUtil.maybeFlipPose(pose);
                    double distance = RobotState.getInstance()
                            .getEstimatedPose()
                            .minus(pose)
                            .getTranslation()
                            .getNorm();
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestFace = pose;
                    }
                }
                return closestFace;
            };
        }
    }

    public static class StagingPositions {
        // Measured from the center of the ice cream
        public static final Pose2d leftIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
        public static final Pose2d middleIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
        public static final Pose2d rightIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }

    public enum ReefHeight {
        L1(Units.inchesToMeters(25.0), 0),
        L2(Units.inchesToMeters(31.875), -35),
        L3(Units.inchesToMeters(47.625), -35),
        L4(Units.inchesToMeters(72), -90);

        ReefHeight(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // in degrees
        }

        public static ReefHeight fromLevel(int level) {
            return values()[MathUtil.clamp(level, 1, 4) - 1];
        }

        public final double height;
        public final double pitch;
    }

    public enum ReefBranch {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L;

        public final Pose2d scorePosition;

        ReefBranch() {
            scorePosition = Reef.scorePositions.get(ordinal());
        }

        public static ReefBranch fromOrdinal(int value) {
            return values()[value];
        }
    }

    public record ReefFace(Pose2d pose, Pose2d leftBranchScore, Pose2d rightBranchScore) {}

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final int aprilTagCount = 22;
    public static final AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
}

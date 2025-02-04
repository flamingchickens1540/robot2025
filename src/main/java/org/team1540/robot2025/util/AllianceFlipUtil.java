package org.team1540.robot2025.util;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
    }

    public static Pose2d maybeFlipPose(Pose2d pose) {
        if (shouldFlip()) return FlippingUtil.flipFieldPose(pose);
        return pose;
    }

    public static Translation2d maybeFlipTranslation(Translation2d translation) {
        if (shouldFlip()) return FlippingUtil.flipFieldPosition(translation);
        return translation;
    }

    public static Rotation2d maybeFlipRotation(Rotation2d rotation) {
        if (shouldFlip()) return FlippingUtil.flipFieldRotation(rotation);
        return rotation;
    }

    public static Rotation2d maybeReverseRotation(Rotation2d rotation) {
        if (shouldFlip()) return rotation.plus(Rotation2d.k180deg);
        return rotation;
    }
}

package org.team1540.robot2025.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class JoystickUtil {
    public static double smartDeadzone(double rawInput, double deadzone) {
        double scalar = 1 / (1 - deadzone);
        if (rawInput > deadzone) {
            return (rawInput - deadzone) * scalar;
        }
        if (rawInput < -deadzone) {
            return (rawInput + deadzone) * scalar;
        }
        return 0;
    }

    public static double squaredSmartDeadzone(double rawInput, double deadzone) {
        double deadzonedInput = smartDeadzone(rawInput, deadzone);
        return Math.copySign(deadzonedInput * deadzonedInput, deadzonedInput);
    }

    public static Translation2d deadzonedJoystickTranslation(double rawX, double rawY, double deadzone) {
        double linearMagnitude = JoystickUtil.smartDeadzone(Math.hypot(rawX, rawY), deadzone);
        if (linearMagnitude < 1e-6) return Translation2d.kZero;
        Rotation2d linearDirection = new Rotation2d(rawX, rawY);
        return new Translation2d(linearMagnitude, linearDirection);
    }

    public static Translation2d squareDeadzonedJoystickTranslation(double rawX, double rawY, double deadzone) {
        double linearMagnitude = JoystickUtil.squaredSmartDeadzone(Math.hypot(rawX, rawY), deadzone);
        if (linearMagnitude < 1e-6) return Translation2d.kZero;
        Rotation2d linearDirection = new Rotation2d(rawX, rawY);
        return new Translation2d(linearMagnitude, linearDirection);
    }
}

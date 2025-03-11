package org.team1540.robot2025.util.math;

public class MathUtils {

    public static int clamp(int value, int bound1, int bound2) {
        if (bound1 > bound2) return Math.max(bound2, Math.min(value, bound1));
        else return Math.max(bound1, Math.min(value, bound2));
    }

    public static double clamp(double value, double bound1, double bound2) {
        if (bound1 > bound2) return Math.max(bound2, Math.min(value, bound1));
        else return Math.max(bound1, Math.min(value, bound2));
    }
}

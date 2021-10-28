package org.firstinspires.ftc.teamcode.lib;

public class Targets {
    // differentiate between high goal and powershot
    public enum TargetType {
        HIGHGOAL,
        OUTER_POWERSHOT,
        CENTER_POWERSHOT,
        INNER_POWERSHOT
    }

    // target x coordinate. Since all targets are on the same line this value is identical for every target
    public static final double targetX = 72;

    // targets y coordinate from high goal (0) to inner most powershot (3). Use negative for red
    public static final double[] targetsY = {35, 21, 13.5, 6};
}

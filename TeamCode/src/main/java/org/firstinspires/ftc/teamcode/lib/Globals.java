package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Globals {

    public enum Alliance {
        RED,
        BLUE
    }

    public static int rings;

    public static boolean autonomous;

    public static Alliance alliance = Alliance.BLUE;

    public static Targets.TargetType currentTargetType = Targets.TargetType.HIGHGOAL;

    public static Vector2d currentTarget = new Vector2d();

    public static AutoAim.Mode currentAimingMode = AutoAim.Mode.ALIGN_TO_POINT;

    public static void setTarget(Targets.TargetType targetType) {
        currentTargetType = targetType;
    }

    public static void updateTarget() {
        int targetYArrayIndex = 0;
        switch (currentTargetType) {
            case HIGHGOAL:
                targetYArrayIndex = 0;
                break;
            case OUTER_POWERSHOT:
                targetYArrayIndex = 1;
                break;
            case CENTER_POWERSHOT:
                targetYArrayIndex = 2;
                break;
            case INNER_POWERSHOT:
                targetYArrayIndex = 3;
                break;
        }
        int mirrorCoefficient = 1;
        if(alliance == Alliance.RED)
            mirrorCoefficient = -1;
        currentTarget = new Vector2d(Targets.targetX, mirrorCoefficient*Targets.targetsY[targetYArrayIndex]);
    }

    public static double highGoalRPM = 5000;

    public static double powerShotRPM = 4100;

    public static double powerShotAutoRPM = 3655;

    public static double aimingHeadingError = 5;

    public static int shots = 0;

    public static double rpmToTicksPerSecond(double rpm, double gearRatio) {
        return rpm * 28 / gearRatio / 60;
    }

    public static double ticksPerSecondToRpm(double tps, double gearRatio) {
        return tps / 28 * gearRatio * 60;
    }
}

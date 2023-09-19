package org.firstinspires.ftc.teamcode.robot.motion;

import com.qualcomm.robotcore.util.Range;

public class MotionUtils {

    // Clip velocity or power.
    public static double clip(double pValue, double pLimit) {
        // Do not let the velocity or power go above the limit (positive) or below the limit (negative).
        return Range.clip(Math.abs(pValue), pLimit, 1.0) * (pValue < 0 ? -1 : 1);
    }
}

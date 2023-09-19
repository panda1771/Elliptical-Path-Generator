package org.firstinspires.ftc.teamcode.robot.motion.drive;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.motion.MotionUtils;
import org.firstinspires.ftc.teamcode.robot.motion.MotorCore;

import java.util.EnumMap;
import java.util.List;

// Linear power ramp-down that starts at a user-specified number of inches
// from the target (converted to motor clicks).

// Velocity values for all motors will be the same only when the angle of motion
// is evenly divisible by 90. In this case all 4 motors are dominant. Otherwise,
// e.g. when the robot is moving in a straight line at a 45-degree angle, there
// will always be two dominant motors and two subordinate motors.

public class StraightDriveRampdown {

    public static final String TAG = "RampDownStraight";
    private final FTCRobot robot;
    private final int rampDownClicks;
    private final List<AutoDrive.DriveMotorData> allMotors;

    private final int dominantMotorIndex;
    private double previousDominantVelocity;

    @SuppressLint("DefaultLocale")
    public StraightDriveRampdown(FTCRobot pRobot, int pRampDownClicks,
                                 List<AutoDrive.DriveMotorData> pAllMotors, int pDominantMotorIndex) {
        robot = pRobot;
        rampDownClicks = Math.abs(pRampDownClicks);
        allMotors = pAllMotors;
        dominantMotorIndex = pDominantMotorIndex;
        previousDominantVelocity = Math.abs(allMotors.get(dominantMotorIndex).getInitialVelocity());

        RobotLogCommon.d(TAG, "Ramp down initial velocity of " + String.format("%.2f", previousDominantVelocity) +
                " starting at " + rampDownClicks + " clicks from the target");
    }

    // Returns the ramp down factor, which starts at 1.0 when the robot is running
    // at the velocity specified in RobotAction.xml, and decreases as the robot nears
    // its target.
    @SuppressLint("DefaultLocale")
    public double rampDown(int pRemainingClicks) {

        EnumMap<DriveTrain.MotorId, Double> newVelocityMap = new EnumMap<>(MotorCore.MotorId.class);

        // Sanity check
        if (pRemainingClicks <= 0.0)
            throw new AutonomousRobotException(TAG, "Remaining clicks " + pRemainingClicks +
                    " may not be <= 0");

        int remainingCLicks = Math.abs(pRemainingClicks); // abs for consistency

        // Sanity check: the remaining number of clicks must be <= the point at which the
        // ramp down is to start.
        if (remainingCLicks > rampDownClicks)
            throw new AutonomousRobotException(TAG, "Remaining clicks " + remainingCLicks +
                    " is > ramp down start point of " + rampDownClicks);

        // Simple linear ramp-down. Cast to double is necessary, otherwise result
        // can be 0 when you don't want it.
        double rampDownFactor = Math.abs(pRemainingClicks / (double) rampDownClicks);

        // Running continuously results in frequent small updates to motor velocity.
        // It's better to step the velocity down by this logic: if the absolute value
        // of the current velocity of any dominant motor is equal to the minimum
        // velocity OR the difference between the current velocity and the previous
        // velocity is less than the minimum velocity step, e.g. .05, then do NOT
        // update the motor velocity.
        double currentDominantVelocity =
                Math.abs(MotionUtils.clip(allMotors.get(dominantMotorIndex).getInitialVelocity() * rampDownFactor, DriveTrainConstants.MINIMUM_DOMINANT_MOTOR_VELOCITY));
        RobotLogCommon.vv(TAG, "Next candidate for velocity ramp-down " + String.format("%.2f", currentDominantVelocity) +
                " using factor " + String.format("%.3f", rampDownFactor) +
                ", previous " + String.format("%.2f", previousDominantVelocity) +
                ", difference " + String.format("%.3f", Math.abs(currentDominantVelocity - previousDominantVelocity)));

        if (currentDominantVelocity == DriveTrainConstants.MINIMUM_DOMINANT_MOTOR_VELOCITY || Math.abs(currentDominantVelocity - previousDominantVelocity) < DriveTrainConstants.MINIMUM_DRIVE_POWER_STEP)
            return rampDownFactor;

        previousDominantVelocity = currentDominantVelocity;
        double clippedVelocity;
        for (AutoDrive.DriveMotorData oneMotor : allMotors) {
            clippedVelocity = (oneMotor.motorRank == DriveTrainConstants.MotorRank.DOMINANT) ? MotionUtils.clip(oneMotor.getInitialVelocity() * rampDownFactor, DriveTrainConstants.MINIMUM_DOMINANT_MOTOR_VELOCITY) :
                    MotionUtils.clip(oneMotor.getInitialVelocity() * rampDownFactor, 0.0);
            newVelocityMap.put(oneMotor.motorId, clippedVelocity);
        }

        robot.driveTrain.driveAllByVelocity(newVelocityMap);

        RobotLogCommon.vv(TAG, "Straight line velocity ramped down to lf " + String.format("%.2f", newVelocityMap.get(DriveTrain.MotorId.LEFT_FRONT_DRIVE)) +
                ", rf " + String.format("%.2f", newVelocityMap.get(DriveTrain.MotorId.RIGHT_FRONT_DRIVE)) +
                ", lb " + String.format("%.2f", newVelocityMap.get(DriveTrain.MotorId.LEFT_BACK_DRIVE)) +
                ", rb " + String.format("%.2f", newVelocityMap.get(DriveTrain.MotorId.RIGHT_BACK_DRIVE)));

        return rampDownFactor;
    }

}

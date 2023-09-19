package org.firstinspires.ftc.teamcode.robot.motion.drive;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.motion.MotionUtils;

import java.util.EnumMap;

// Linear power ramp-down that starts at a user-specified number of degrees.
public class TurnRampdown {

    public static final String TAG = "RampDownTurn";
    private final FTCRobot robot;
    private final double rampDownDegrees;
    private double previousPower;
    private final EnumMap<DriveTrain.MotorId, Double> currentPowerMap;

    @SuppressLint("DefaultLocale")
    public TurnRampdown(FTCRobot pRobot, double pRampDownDegrees, EnumMap<DriveTrain.MotorId, Double> pCurrentPowerMap) {
        robot = pRobot;
        rampDownDegrees = Math.abs(pRampDownDegrees);
        currentPowerMap = pCurrentPowerMap;

        // Power values for all motors are the same (although their signs may be different).
        // Choose any motor to set the previous power.
        previousPower = Math.abs(currentPowerMap.get(DriveTrain.MotorId.LEFT_FRONT_DRIVE));

        RobotLogCommon.d(TAG, "Ramp down turn power starting at " + String.format("%.2f", rampDownDegrees) +
                " degrees remaining and " + String.format("%.2f", previousPower) + " power");
    }

    @SuppressLint("DefaultLocale")
    public void rampDown(double pRemainingAngle) {

        EnumMap<DriveTrain.MotorId, Double> newPowerMap = new EnumMap<>(DriveTrain.MotorId.class);

        // Sanity check: the remaining angle must be <= the point at which the ramp down
        // is to start.
        if (Math.abs(pRemainingAngle) > rampDownDegrees)
            throw new AutonomousRobotException(TAG, "Remaining angle of " + String.format("%.2f", pRemainingAngle) +
                    " is > ramp down start point of " + String.format("%.2f", rampDownDegrees));

        // Simple linear ramp-down.
        double rampDownFactor = Math.abs(pRemainingAngle / rampDownDegrees);

        // Running continuously results in frequent small updates to motor power. It's
        // better to step the power down by this logic: if the absolute value of the
        // current power (of any motor, all are the same) is equal to the minimum
        // power OR the difference between the current power and the previous power
        // is less than the minimum power step, e.g. .05, then do NOT update the power
        // to the motors.
        double currentPower = Math.abs(MotionUtils.clip(currentPowerMap.get(DriveTrain.MotorId.LEFT_FRONT_DRIVE) * rampDownFactor, DriveTrainConstants.MINIMUM_TURN_POWER));
        if (currentPower == DriveTrainConstants.MINIMUM_TURN_POWER || Math.abs(currentPower - previousPower) < DriveTrainConstants.MINIMUM_TURN_POWER_STEP)
            return;

        previousPower = currentPower;
        double clippedPower;
        for (EnumMap.Entry<DriveTrain.MotorId, Double> oneMotor : currentPowerMap.entrySet()) {
            clippedPower = MotionUtils.clip(oneMotor.getValue() * rampDownFactor, DriveTrainConstants.MINIMUM_TURN_POWER);
            newPowerMap.put(oneMotor.getKey(), clippedPower);
        }

        robot.driveTrain.driveAllByPower(newPowerMap);

        RobotLogCommon.v(TAG, "Turn power lf " + String.format("%.2f", newPowerMap.get(DriveTrain.MotorId.LEFT_FRONT_DRIVE)) +
                ", rf " + String.format("%.2f", newPowerMap.get(DriveTrain.MotorId.RIGHT_FRONT_DRIVE)) +
                ", lb " + String.format("%.2f", newPowerMap.get(DriveTrain.MotorId.LEFT_BACK_DRIVE)) +
                ", rb " + String.format("%.2f", newPowerMap.get(DriveTrain.MotorId.RIGHT_BACK_DRIVE)));
    }

}

package org.firstinspires.ftc.teamcode.robot.motion.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.util.EnumMap;
import java.util.List;

public class DriveTrainMotion {

    private static final String TAG = "RobotMotion";

    private final LinearOpMode linearOpMode;
    private final FTCRobot robot;

    public DriveTrainMotion(LinearOpMode pLinearOpMode, FTCRobot pRobot) {
        linearOpMode = pLinearOpMode;
        robot = pRobot;
    }

    // Drive the robot in a straight line while maintaining its heading.
    // See AutoDriveCore.java for an explanation of "dominant" and
    // "subordinate" motors. But an easy point to remember is that if
    // the angle of the robot's movement is evenly divisible by 90 degrees
    // then all 4 motors are dominant and all 4 run at the same velocity.

    // For all other angles, 2 motors will be dominant and 2 will be
    // subordinate. For subordinate motors set the default mode to
    // RUN_USING_ENCODER. Experiments showed that if all motors are set
    // to RUN_TO_POSITION the subordinate motors continue to run even
    // after the velocity of the dominant motors is set to 0.
    @SuppressLint("DefaultLocale")
    public void straight(int pTargetClicks, double pAngle, double pVelocity,
                         int pRampDownAtClicksRemaining, double pDesiredHeading) {

        int targetClicks = Math.abs(pTargetClicks); // ensure consistency
        int rampDownAtClicksRemaining = Math.abs(pRampDownAtClicksRemaining); // ensure consistency

        RobotLogCommon.d(TAG, "Start straight drive at angle " + String.format("%.2f", pAngle) +
                ", ramp down at abs " + rampDownAtClicksRemaining +
                " clicks remaining, desired heading " + String.format("%.2f", pDesiredHeading));

        RobotLogCommon.d(TAG, "Straight line velocity " + pVelocity);
        AutoDrive adc = new AutoDrive(pAngle, pVelocity, DriveTrainConstants.MINIMUM_DOMINANT_MOTOR_VELOCITY);
        List<AutoDrive.DriveMotorData> allDriveMotors = adc.getDriveMotorData();

        // Set the run mode for all motors.
        //## Follow the FTC sample PushbotAutoDriveByEncoder_Linear and always
        // set the run modes in this order: STOP_AND_RESET_ENCODER,
        // RUN_USING_ENCODER. Then, if running to a position, call
        // setTargetPosition followed by a run mode of RUN_TO_POSITION.
        EnumMap<DriveTrain.MotorId, Double> velocityMap = new EnumMap<>(DriveTrain.MotorId.class);
        robot.driveTrain.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLogCommon.d(TAG, "Initial motor settings");
        for (AutoDrive.DriveMotorData oneMotor : allDriveMotors) {

            // Only dominant motors will use RUN_TO_POSITION.
            // But first, now that we know the click count, we set the target position.
            if (oneMotor.motorRank == DriveTrainConstants.MotorRank.DOMINANT) {
                robot.driveTrain.setTargetPosition(oneMotor.motorId, targetClicks * oneMotor.directionSignum);
                robot.driveTrain.setMode(oneMotor.motorId, DcMotor.RunMode.RUN_TO_POSITION);
            }

            velocityMap.put(oneMotor.motorId, oneMotor.getInitialVelocity());
            RobotLogCommon.d(TAG, "Motor " + oneMotor.motorId + ", mode " + oneMotor.runMode +
                    ", " + oneMotor.motorRank +
                    ", clicks " + targetClicks * oneMotor.directionSignum +
                    ", velocity " + String.format("%.2f", oneMotor.getInitialVelocity()));
        }

        //**TODO this looks lame. Why pass the index around - just get the motorId
        // of the first dominant motor. Use a stream; filter by dominant motor,
        // take the first motorId.
        // Get the index of the first dominant motor.
        int dominantMotorIndex;
        for (dominantMotorIndex = 0; dominantMotorIndex < allDriveMotors.size(); dominantMotorIndex++) {
            if (allDriveMotors.get(dominantMotorIndex).motorRank == DriveTrainConstants.MotorRank.DOMINANT)
                break;
        }

        StraightDriveRampdown straightDriveRampdown = new StraightDriveRampdown(robot, rampDownAtClicksRemaining, allDriveMotors, dominantMotorIndex);

        // Start moving.
        robot.driveTrain.driveAllByVelocity(velocityMap);
        DriveTrainPID driveTrainPID = new DriveTrainPID(DriveTrainConstants.P_DRIVE_COEFF);

        // Keep moving until one of the dominant motors has reached its target
        // position.
        try {
            double currentHeading;
            int dominantMotorClickCount;
            int remainingClickCount;
            double rampDownFactor = 1.0; // start with no ramp-down
            boolean robotReachedTarget = false;
            while (!robotReachedTarget) {
                if (!linearOpMode.opModeIsActive()) {
                    RobotLogCommon.d(TAG, "OpMode went inactive during straight line run");
                    break;
                }

                robotReachedTarget = allDriveMotors.stream()
                        .filter(m -> m.motorRank == DriveTrainConstants.MotorRank.DOMINANT)
                        .anyMatch(m -> !robot.driveTrain.isBusy(m.motorId));
                if (robotReachedTarget)
                    continue; // and exit while loop

                currentHeading = robot.imu.getIMUHeading();
                applyStraightLinePID(pDesiredHeading, currentHeading, allDriveMotors,
                        driveTrainPID, rampDownFactor);

                //**TODO ??rampdown may have been already applied in the PID
                // method. But watch out,
                // the PID method only changes motor velocity if steer >=
                // MINIMUM_DRIVE_POWER_STEP. ??Flag and enum - if PID has applied
                // the rampdown, do not reapply in straightDriveRampdown.rampDown,
                // - just return the new value.

                // Check for requested velocity ramp-down.
                // Get the click count of one of the dominant motors.
                dominantMotorClickCount = Math.abs(robot.driveTrain.getClickCount(allDriveMotors.get(dominantMotorIndex).motorId));
                remainingClickCount = targetClicks - dominantMotorClickCount;
                if (rampDownAtClicksRemaining != 0.0 &&
                        dominantMotorClickCount < targetClicks &&
                        remainingClickCount <= rampDownAtClicksRemaining)
                    rampDownFactor = straightDriveRampdown.rampDown(remainingClickCount);
            } // while
        } finally {
            robot.driveTrain.stopAllZeroVelocity();

            // Log ending click counts for all dominant motors.
            RobotLogCommon.d(TAG, "Straight line drive complete");
            allDriveMotors.forEach(m -> {
                        if (m.motorRank == DriveTrainConstants.MotorRank.DOMINANT)
                            RobotLogCommon.d(TAG, "Dominant motor " + m.motorId +
                                    " ending click count " + robot.driveTrain.getClickCount(m.motorId));
                    }
            );
        }
    }

    // Executes a turn.
    // Takes into account the fact that the desired heading of the robot before the turn may not be the same
    // as the current heading. So this method either increases or diminishes the turn to make up the gap.
    // Returns the desired heading after the turn.
    // Honors the pStartRampDown parameter, which tells this method at what point (in degrees) to start
    // ramping down the power as a way of avoiding overturning.
    //?? Import stall detection from Skystone Summer 2020 if needed.
    @SuppressLint("DefaultLocale")
    public double turn(double pDesiredHeadingBeforeTurn, double pTurnDegrees, double pPower, double pStartRampDownDegrees, DriveTrainConstants.TurnType pTurnType) {

        double startRampDown = Math.abs(pStartRampDownDegrees); // abs for consistency

        RobotLogCommon.d(TAG, "Start turn, desired heading before turn " + pDesiredHeadingBeforeTurn +
                ", requested turn angle " + String.format("%.2f", pTurnDegrees) +
                ", power " + String.format("%.2f", pPower) +
                ", start ramp down at " + startRampDown + " degrees remaining, turn type " + pTurnType);

        robot.driveTrain.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // the IMU controls the turn

        double currentHeading = robot.imu.getIMUHeading();
        Headings.TurnData turnData = Headings.getActualTurn(pDesiredHeadingBeforeTurn, currentHeading, pTurnDegrees, pTurnType);
        RobotLogCommon.d(TAG, "Start turn: current heading " + String.format("%.2f", currentHeading) +
                ", angle to turn " + String.format("%.2f", turnData.actualTurn));
        RobotLogCommon.d(TAG, "Desired heading after turn " + String.format("%.2f", turnData.desiredHeadingAfterTurn));

        // Sanity check: nothing to do for a turn of 0 degrees.
        if (turnData.actualTurn == 0.0)
            return pDesiredHeadingBeforeTurn;

        // A CW turn requires positive power to the left side motors and
        // negative power to the right side motors.
        double power = Math.abs(pPower); // start with positive power
        EnumMap<DriveTrain.MotorId, Double> powerMap = new EnumMap<>(DriveTrain.MotorId.class);
        powerMap.put(DriveTrain.MotorId.LEFT_FRONT_DRIVE, power * Math.signum(turnData.actualTurn) * -1);
        powerMap.put(DriveTrain.MotorId.RIGHT_FRONT_DRIVE, power * Math.signum(turnData.actualTurn));
        powerMap.put(DriveTrain.MotorId.LEFT_BACK_DRIVE, power * Math.signum(turnData.actualTurn) * -1);
        powerMap.put(DriveTrain.MotorId.RIGHT_BACK_DRIVE, power * Math.signum(turnData.actualTurn));

        // Set up for ramp down.
        TurnRampdown turnRampdown = new TurnRampdown(robot, startRampDown, powerMap);

        // Start turn.
        robot.driveTrain.driveAllByPower(powerMap);

        try {
            // Keep looping while we are still active and have not yet reached the desired heading.
            double previousCurrentHeading;
            double degreeDifference;
            double degreesTurned = 0;
            double remainingAngle;
            boolean turnComplete = false;
            while (!turnComplete) {
                if (!linearOpMode.opModeIsActive()) {
                    RobotLogCommon.d(TAG, "OpMode went inactive during turn");
                    break;
                }

                // Verified calculation of remaining angle taken from HeadingUtils::simulateTurn
                // in Visual Studio project AngleCalculator.

                // If the robot has reached the turn window, e.g. 2 degrees, stop here.
                // Otherwise keep turning.
                previousCurrentHeading = currentHeading;
                currentHeading = robot.imu.getIMUHeading();
                degreeDifference = Headings.normalize(currentHeading - previousCurrentHeading, -180, 180);
                degreesTurned += degreeDifference;
                remainingAngle = turnData.actualTurn - degreesTurned;
                //##!! floods log RobotLogCommon.vv(TAG, "Current heading " + currentHeading + " remaining angle " + remainingAngle);

                if (Math.abs(remainingAngle) <= DriveTrainConstants.TURN_THRESHOLD_DEGREES) {
                    RobotLogCommon.d(TAG, "Reached turn threshold; turn complete");
                    RobotLogCommon.d(TAG, "Current heading at turn complete " + String.format("%.2f", currentHeading));
                    turnComplete = true;
                    continue;
                }

                // Make sure that the sign of the remaining angle is the same as that of
                // the original turn.
                if (Math.signum(remainingAngle) != Math.signum(turnData.actualTurn)) {
                    RobotLogCommon.d(TAG, "Overshot turn: remaining angle " + String.format("%.2f", remainingAngle));
                    turnComplete = true;
                    continue;
                }

                // Check for a ramp-down in power as the robot approaches its target.
                if (startRampDown != 0.0 && (Math.abs(remainingAngle) <= startRampDown))
                    turnRampdown.rampDown(remainingAngle);
            }

            return turnData.desiredHeadingAfterTurn;
        } // try

        // In case of any unforseen conditions always stop the motors.
        finally {
            robot.driveTrain.stopAllZeroPower();
            RobotLogCommon.d(TAG, "IMU heading after turn " + String.format("%.2f", robot.imu.getIMUHeading()));
        }
    }

    // Applies a PID and adjusts the velocity of each motor.
    // Running continuously results in frequent small updates to motor velocity.
    // It's better to adjust the velocity only when the absolute value of the
    // output of the PID (the "steer" value) is equal to or greater than the
    // minimum power step, e.g. .05. The parameter pRampDownFactor is included
    // so that the PID can be applied as the robot's velocity ramps down
    // according to the optional value in RobotAction.xml.
    @SuppressLint("DefaultLocale")
    private void applyStraightLinePID(double pDesiredHeading, double pCurrentHeading,
                                      List<AutoDrive.DriveMotorData> pCurrentMotorData,
                                      DriveTrainPID pPID, double pRampDownFactor) {
        EnumMap<DriveTrain.MotorId, Double> newVelocityMap = new EnumMap<>(DriveTrain.MotorId.class);

        double error = DEGREES.normalize(pDesiredHeading - pCurrentHeading);
        RobotLogCommon.vv(TAG, "IMU " + String.format("%.2f", pCurrentHeading) +
                ", error " + String.format("%.2f", error));
        double steer = pPID.getPIDValue(error);

        if (Math.abs(steer) < DriveTrainConstants.MINIMUM_DRIVE_POWER_STEP)
            return; // velocity increment too small, skip

        double clippedVelocity;
        for (AutoDrive.DriveMotorData oneMotor : pCurrentMotorData) {
            switch (oneMotor.motorId) {
                case LEFT_FRONT_DRIVE:
                case LEFT_BACK_DRIVE: {
                    double newLeftVelocity = (oneMotor.getInitialVelocity() * pRampDownFactor) - steer;
                    clippedVelocity = oneMotor.clipUpdatedVelocity(newLeftVelocity);
                    newVelocityMap.put(oneMotor.motorId, clippedVelocity);
                    break;
                }
                case RIGHT_FRONT_DRIVE:
                case RIGHT_BACK_DRIVE: {
                    double newRightVelocity = (oneMotor.getInitialVelocity() * pRampDownFactor) + steer;
                    clippedVelocity = oneMotor.clipUpdatedVelocity(newRightVelocity);
                    newVelocityMap.put(oneMotor.motorId, clippedVelocity);
                    break;
                }
                default:
                    throw new AutonomousRobotException(TAG, "Invalid motor position " + oneMotor.motorId);
            }
        }

        robot.driveTrain.driveAllByVelocity(newVelocityMap);

        RobotLogCommon.v(TAG, "Straight velocity lf " + String.format("%.2f", newVelocityMap.get(DriveTrain.MotorId.LEFT_FRONT_DRIVE)) +
                ", rf " + String.format("%.2f", newVelocityMap.get(DriveTrain.MotorId.RIGHT_FRONT_DRIVE)) +
                ", lb " + String.format("%.2f", newVelocityMap.get(DriveTrain.MotorId.LEFT_BACK_DRIVE)) +
                ", rb " + String.format("%.2f", newVelocityMap.get(DriveTrain.MotorId.RIGHT_BACK_DRIVE)));
    }

}

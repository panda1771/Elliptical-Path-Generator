package org.firstinspires.ftc.teamcode.robot.motion.drive;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.motion.MotionUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

// For managing the motors during a straight run in Autonomous.
public class AutoDrive {

    //**TODO Consider pulling motorId out of DriveMotorData and making
    // it a key into an EnumMap. This is more consistent with all other
    // uses. The position of a motor in the list is not significant.
    private final List<DriveMotorData> allDriveMotors;

    // For a straight run in Autonomous we always use RUN_WITH_ENCODER
    // with velocity levels with or without RUN_TO_POSITION.

    // Directional velocity levels, including x, y, and rotational,
    // make sense in TeleOp where the driver can stop the robot
    // using visual cues. But in autonomous we use distance
    // (converted to encoder clicks for each motor) to know when a
    // movement is complete. To ask the user to predict click counts
    // in the presence of rotation is not practical. Full freedom of
    // movement in autonomous is best left to a package like
    // RoadRunner.
    @SuppressLint("DefaultLocale")
    public AutoDrive(double pFtcAngle, double pVelocity, double pDominantMotorVelocityLimit) {

        double dominantVelocityLimit = Math.abs(pDominantMotorVelocityLimit);

        // To ensure that the trigonometry produces values that match the
        // FTC convention (CCW 0 to +180 not inclusive, CW 0 to -180
        // inclusive), it is necessary to invert the FTC angle and
        // then convert it to a 0..360 CW range.
        double ftcAngle = DEGREES.normalize(pFtcAngle); // ensure correct range
        double angle360 = Headings.normalize(ftcAngle * -1, 0, 360);

        // Get the directional values.
        double directionX = Math.sin(Math.toRadians(angle360));
        double directionY = Math.cos(Math.toRadians(angle360));

        // Calculate the unclipped, unfactored directional velocity
        // for the mecanum drive motors.
        //**TODO need better naming/comments. These values are used
        // to set the initial velocity and also determine the direction
        // signum below.
        double lfv = directionY + directionX;
        double rfv = directionY - directionX;
        double lbv = directionY - directionX;
        double rbv = directionY + directionX;

        // Determine which motors have the dominant velocity values.
        // The sign of the velocity for each motor can be used to
        // to determine the sign of the target click count for
        // RUN_TO_POSITION. Also, only the dominant motors need to
        // be checked for isBusy() during RUN_TO_POSITION.

        // Template for dominant motors.
        DcMotor.RunMode dominantRunMode = DcMotor.RunMode.RUN_TO_POSITION;
        DriveTrainConstants.MotorRank dominantMotorRank = DriveTrainConstants.MotorRank.DOMINANT;

        // For forward (0 degrees), backward (-180 degrees), strafe left
        // (90 degrees), and strafe right (-90 degrees) all motors will be
        // in play, all will be dominant, and all will have the same velocity.

        allDriveMotors = new ArrayList<>();
        if (angle360 % 90.0 == 0.0) {
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.LEFT_FRONT_DRIVE, lfv, pVelocity, dominantVelocityLimit, dominantRunMode, dominantMotorRank));
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.RIGHT_FRONT_DRIVE, rfv, pVelocity, dominantVelocityLimit, dominantRunMode, dominantMotorRank));
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.LEFT_BACK_DRIVE, lbv, pVelocity, dominantVelocityLimit, dominantRunMode, dominantMotorRank));
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.RIGHT_BACK_DRIVE, rbv, pVelocity, dominantVelocityLimit, dominantRunMode, dominantMotorRank));
            return;
        }

        // For all other angles, two of the four motors will be dominant.
        List<Pair<DriveTrain.MotorId, Double>> tempSort = Arrays.asList(
                Pair.create(DriveTrain.MotorId.LEFT_FRONT_DRIVE, lfv),
                Pair.create(DriveTrain.MotorId.RIGHT_FRONT_DRIVE, rfv),
                Pair.create(DriveTrain.MotorId.LEFT_BACK_DRIVE, lbv),
                Pair.create(DriveTrain.MotorId.RIGHT_BACK_DRIVE, rbv));

        // Sort by the absolute value of the velocity in descending order to put the
        // dominant motors at the front of the list.
        // See https://stackoverflow.com/questions/16252269/how-to-sort-an-arraylist; 148
        tempSort.sort(
                (Pair<DriveTrain.MotorId, Double> m1,
                 Pair<DriveTrain.MotorId, Double> m2) ->
                        // invert argument order for descending
                        Double.compare(Math.abs(m2.second), Math.abs(m1.second)));

        // Template for subordinate motors.
        DcMotor.RunMode subordinateRunMode= DcMotor.RunMode.RUN_USING_ENCODER;
        DriveTrainConstants.MotorRank subordinateMotorRank = DriveTrainConstants.MotorRank.SUBORDINATE;
        double subordinateVelocityLimit = 0.0;

        // Get the first raw dominant velocity that pops up. Below, if the absolute value
        // is > 1 then reduce the subordinate value proportionally. The dominant value
        // will be clipped to 1 eventually.
        double dominantRawVelocity = Math.abs(tempSort.get(0).second);

        // Mark the dominant motors as RUN_TO_POSITION. These can be checked for
        // isBusy(). Mark the subordinate motors as RUN_USING_ENCODER.
        if (tempSort.get(0).first == DriveTrain.MotorId.LEFT_FRONT_DRIVE ||
                tempSort.get(1).first == DriveTrain.MotorId.LEFT_FRONT_DRIVE)
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.LEFT_FRONT_DRIVE, lfv, pVelocity, dominantVelocityLimit, dominantRunMode, dominantMotorRank));
        else
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.LEFT_FRONT_DRIVE, (dominantRawVelocity <= 1.0 ? lfv : lfv/dominantRawVelocity), pVelocity, subordinateVelocityLimit, subordinateRunMode, subordinateMotorRank));

        if (tempSort.get(0).first == DriveTrain.MotorId.RIGHT_FRONT_DRIVE ||
                tempSort.get(1).first == DriveTrain.MotorId.RIGHT_FRONT_DRIVE)
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.RIGHT_FRONT_DRIVE, rfv, pVelocity, dominantVelocityLimit, dominantRunMode, dominantMotorRank));
        else
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.RIGHT_FRONT_DRIVE, (dominantRawVelocity <= 1.0 ? rfv : rfv/dominantRawVelocity), pVelocity, subordinateVelocityLimit, subordinateRunMode, subordinateMotorRank));

        if (tempSort.get(0).first == DriveTrain.MotorId.LEFT_BACK_DRIVE ||
                tempSort.get(1).first == DriveTrain.MotorId.LEFT_BACK_DRIVE)
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.LEFT_BACK_DRIVE, lbv, pVelocity, dominantVelocityLimit, dominantRunMode, dominantMotorRank));
        else
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.LEFT_BACK_DRIVE, (dominantRawVelocity <= 1.0 ? lbv : lbv/dominantRawVelocity), pVelocity, subordinateVelocityLimit, subordinateRunMode, subordinateMotorRank));

        if (tempSort.get(0).first == DriveTrain.MotorId.RIGHT_BACK_DRIVE ||
                tempSort.get(1).first == DriveTrain.MotorId.RIGHT_BACK_DRIVE)
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.RIGHT_BACK_DRIVE, rbv, pVelocity, dominantVelocityLimit, dominantRunMode, dominantMotorRank));
        else
            allDriveMotors.add(new DriveMotorData(DriveTrain.MotorId.RIGHT_BACK_DRIVE, (dominantRawVelocity <= 1.0 ? rbv : rbv/dominantRawVelocity), pVelocity, subordinateVelocityLimit, subordinateRunMode, subordinateMotorRank));
    }

    public List<DriveMotorData> getDriveMotorData() {
        return allDriveMotors;
    }

    // Very important!
    // For motors set to RUN_TO_POSITION, the SDK does not look at
    // the sign of the velocity. However, for the PID control to
    // work correctly we do need the sign of the velocity. So if
    // the motor is a dominant motor, i.e. its run mode is
    // RUN_TO_POSITION, then do not allow its velocity to fall
    // outside the specified limit in either a positive or negative
    // direction. But the velocity of a non-dominant motor may
    // legitimately fall below the limit: for example, for a
    // 45-degree NE strafe the velocity of the right front motor is
    // 0. But we do not allow a sign flip by setting the clipping
    // limit for non-dominant motors to 0.0.
    public static class DriveMotorData {
        private static final String TAG = "MotorData";

        public final DriveTrain.MotorId motorId;
        public final int directionSignum;
        public final DcMotor.RunMode runMode;
        public final DriveTrainConstants.MotorRank motorRank;
        private final double velocityLimit;
        private final double initialVelocity;

        @SuppressLint("DefaultLocale")
        public DriveMotorData(DriveTrain.MotorId pMotorId, double pDirectionalVelocity, double pVelocity, double pVelocityLimit,
                              //**TODO - run mode can be removed - see RobotMotion
                              DcMotor.RunMode pRunMode,
                              DriveTrainConstants.MotorRank pMotorRank) {
            motorId = pMotorId;
            directionSignum = (int) Math.signum(pDirectionalVelocity);
            velocityLimit = pVelocityLimit;
            runMode = pRunMode;
            motorRank = pMotorRank;

            String directionalVelocityLabel = motorRank == DriveTrainConstants.MotorRank.DOMINANT ? " raw" : " factored";
            RobotLogCommon.d(TAG, pMotorId + directionalVelocityLabel + " directional velocity " + String.format("%.3f", pDirectionalVelocity));
            RobotLogCommon.d(TAG, pMotorId + " velocity factor " + String.format("%.3f", pVelocity));

            double clippedDirectionalVelocity = MotionUtils.clip(pDirectionalVelocity, velocityLimit);
            RobotLogCommon.d(TAG, pMotorId + " clipped directional velocity " + String.format("%.3f", clippedDirectionalVelocity));
            initialVelocity = MotionUtils.clip(clippedDirectionalVelocity * pVelocity, velocityLimit);
            RobotLogCommon.d(TAG, pMotorId + " calculated initial velocity " + String.format("%.3f", initialVelocity) + " with limit " + String.format("%.3f", velocityLimit));
        }

        public double getInitialVelocity() {
            return initialVelocity;
        }

        public double clipUpdatedVelocity(double pVelocity) {
            return MotionUtils.clip(pVelocity, velocityLimit);
        }
    }

}

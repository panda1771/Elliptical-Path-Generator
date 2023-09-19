package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;


import org.firstinspires.ftc.teamcode.robot.motion.MotionUtils;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

public abstract class DriveBase extends TeleOpBase {

    protected void updateDrive(double pVelocityFactor) {
        // up on the gamepad stick is negative
        // change to straight power from Math.pow(gamepad1..., 3);
        double directionX = gamepad1.left_stick_x;
        double directionY = -gamepad1.left_stick_y;
        double directionR = gamepad1.right_stick_x;

        double lfv = MotionUtils.clip(directionY + directionR + directionX, 0.0);
        double rfv = MotionUtils.clip(directionY - directionR - directionX, 0.0);
        double lbv = MotionUtils.clip(directionY + directionR - directionX, 0.0);
        double rbv = MotionUtils.clip(directionY - directionR + directionX, 0.0);
        robot.driveTrain.driveAllByVelocity(lfv * pVelocityFactor, rfv * pVelocityFactor, lbv * pVelocityFactor, rbv * pVelocityFactor);
    }

}
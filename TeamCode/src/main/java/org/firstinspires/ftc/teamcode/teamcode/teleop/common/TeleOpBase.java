package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

public abstract class TeleOpBase extends LinearOpMode {

    private static final String TAG = "TeleOpBase";
    protected FTCRobot robot;

    protected abstract void initialize();

    protected abstract void run() throws InterruptedException;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing...", "Please wait until complete");
        telemetry.update();

        try {
            RobotLogCommon.initialize(WorkingDirectory.getWorkingDirectory() + RobotConstants.logDir);

            robot = new FTCRobot(this);
            initialize(); // give derived class a chance to initialize

            telemetry.addData("Initialized!", "Ready to run");
            telemetry.update();

            waitForStart();
            run(); // run derived class
        } catch (Exception ex) {
            FTCErrorHandling.handleFtcErrors(ex, TAG, this);
        } finally {
            RobotLogCommon.closeLog();
        }
    }
}

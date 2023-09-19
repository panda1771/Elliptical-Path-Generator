package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;

@Autonomous(name = "Test", group = "TeamCode")
//@Disabled
public class Test extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        FTCAutoDispatch dispatch = new FTCAutoDispatch();
        dispatch.runOpMode(RobotConstantsFreightFrenzy.OpMode.TEST, RobotConstants.Alliance.RED, this);
    }
}



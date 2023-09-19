package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXMLFreightFrenzy;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.RobotActionTeleOp;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;
import javax.xml.xpath.XPathExpressionException;

@TeleOp(group = "Drive")
//@Disabled
public class BasicDrive extends DriveBase {

    private static final String TAG = "BasicDrive";

    private final FTCButton imuReadRateButton = new FTCButton();
    private final FTCButton insertAutonomousActionButton = new FTCButton();

    // Fields extracted from RobotConfig.xml.
    //## This is a how-to example. See initialize() below.
    // private double highGoalShootVelocity;

    // For reading RobotAction.xml in TeleOp.
    private RobotActionTeleOp teleOpActions;
    private RobotActionXMLFreightFrenzy.RobotActionDataFreightFrenzy autoDriveWithinTeleOp;

    @Override
    public void initialize() { // can't throw XML errors here because this is an override

        // Get parameters from robotConfig.xml.
        //## This is an example that shows how to get a value, in this case a
        //## motor velocity, from RobotConfig.xml.
        /*
        XPathAccess config = robot.configXML.getPath("FULL_DRIVE");
        try {
            highGoalShootVelocity = config.getDouble("high_goal", 0);
        } catch (XPathExpressionException e) {
            throw new AutonomousRobotException(TAG, "XPath error in initialize");
        }
        */

        // Get pre-canned actions from RobotAction.xml.
        try {
            String xmlDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir;

            //## This is a demonstration of how to run autonomous actions from within TeleOp.
            // This feature can be useful, for example, in a shooting sequence where the driver
            // positions the robot and then pushes a button that automatically shoots, turns a
            // few degrees, and shoots again.

            //## As a demo we'll show how to run a set of motion actions such as drive forward
            // and turn when the TeleOp driver pushes a button on the game controller.

            //## 1. Parse the RobotAction.xml.
            RobotActionXMLFreightFrenzy actionXML = new RobotActionXMLFreightFrenzy(xmlDirectory);

            //## 2. Identify the set of actions as an OpMode in the RobotAction.xml file.
            // The OpMode must have an enum value in RobotConstantsUltimateGoal.OpMode.
            autoDriveWithinTeleOp = actionXML.getOpModeData(RobotConstantsFreightFrenzy.OpMode.TELEOP_AUTO_DRIVE.toString());

            //## 3. Any actions called out in the XML file must be supported in the
            // class RobotActionTeleOp. See that class to see how to add actions.
            teleOpActions = new RobotActionTeleOp(this, robot);

            //## 4. See below for an example of executing the autonomous actions when Player
            // 1 pushes a button on the game controller.
        } catch (IOException | SAXException | ParserConfigurationException | XPathExpressionException ex) {
            throw new AutonomousRobotException(TAG, ex.getMessage());
        }
    }

    @Override
    public void run() throws InterruptedException {
        while (opModeIsActive()) {
            updateButtons();
            updatePlayerOne();
            updatePlayerTwo();
        }
    }

    // Update the state of the active buttons. This method should be
    // called once per cycle.
    private void updateButtons() {
        imuReadRateButton.update(gamepad1.right_bumper);
        insertAutonomousActionButton.update(gamepad1.left_bumper);
    }

    // Execute the action(s) controlled by Player 1.  This method
    // should be called once per cycle.
    private void updatePlayerOne() throws InterruptedException {
        updateDrive(1.0);
        updateImuReadRate();
        insertAutoDrive();
    }

    private void updatePlayerTwo() {
        // Placeholder
    }

    // For testing IMU throughput.
    private void updateImuReadRate() {
        if (imuReadRateButton.is(FTCButton.State.TAP) || imuReadRateButton.is(FTCButton.State.HELD)) {
            int imuReadRate = (int) robot.imu.getIMUReadRate(); // truncate
            telemetry.addData("IMU read rate per second", imuReadRate);
            telemetry.update();
        }
    }

    //## 4. (continued) Execute the autonomous actions when the button is pushed.
    private void insertAutoDrive() throws InterruptedException {
        if (insertAutonomousActionButton.is(FTCButton.State.TAP)) {
            try {
                telemetry.addData("Auto drive within TeleOp", "start");
                telemetry.update();
                teleOpActions.actionLoop(autoDriveWithinTeleOp.actions);
                telemetry.addData("Auto drive within TeleOp", "end");
                telemetry.update();
            } catch (IOException | XPathException ex) {
                throw new AutonomousRobotException(TAG, ex.getMessage());
            }
        }
    }

}

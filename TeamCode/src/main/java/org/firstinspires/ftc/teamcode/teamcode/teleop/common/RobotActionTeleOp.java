package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.motion.drive.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.robot.motion.drive.DriveTrainMotion;

import java.io.IOException;
import java.util.List;

import javax.xml.xpath.XPathException;
import static android.os.SystemClock.sleep;

public class RobotActionTeleOp {

    private static final String TAG = "RobotActionTeleOp";

    private final FTCRobot robot;
    private final LinearOpMode linearOpMode;
    private final DriveTrainMotion motion;
    private double desiredHeading = 0.0; // always normalized

    // For TeleOp: class that executes actions from RobotAction.xml.
    // Defiine a series of actions to a pseudo OpMode and execute the actions
    // in TeleOp when a button is pressed.
    public RobotActionTeleOp(LinearOpMode pLinearOpMode, FTCRobot pRobot) {
        linearOpMode = pLinearOpMode;
        robot = pRobot;
        motion = new DriveTrainMotion(linearOpMode, robot);
    }

    //===============================================================================================
    //===============================================================================================

    // Follow the choreography specified in the robot action file.
    public void actionLoop(List<RobotXMLElement> pActions) throws XPathException, IOException {
        // Always set the desired heading to wherever the robot is pointing at the time
        // this command loop starts.
        desiredHeading = robot.imu.getIMUHeading();

        for (RobotXMLElement action : pActions) {
            executeAction(action);
        }
    }

    // Using the XML elements and attributes from the configuration file RobotAction.xml,
    // execute the action.
    private void executeAction(RobotXMLElement pAction) throws XPathException {

        // Set up XPath access to the current action command.
        XPathAccess commandXPath = new XPathAccess(pAction);
        String commandName = pAction.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing robot action " + commandName);

        switch (commandName) {

            // Autonomous actions supported in TeleOp go here
            //!! WARNING: Except for trivial cases such as SLEEP, do not copy/paste from Autonomous.
            //!! Call static methods in FTCAuto instead. Make new ones as needed.

            // Specialization of STRAIGHT_BY.
            case "FORWARD": {
                FTCAuto.straight_by(commandXPath, robot.driveTrain, motion, 0.0, desiredHeading);
                break;
            }

            // A normalized turn, i.e. a turn from 0 to +180 (not inclusive)
            // degrees CCW and 0 to -180 degrees CW, which will always be the
            // shortest distance from the current heading to the desired heading.
            case "TURN": {
                double angle = commandXPath.getDouble("angle");
                if ((angle > 0 && angle < 180) || (angle < 0 && angle >= -180))
                    desiredHeading = FTCAuto.turn(commandXPath, motion, desiredHeading, angle, DriveTrainConstants.TurnType.NORMALIZED);
                else
                    throw new AutonomousRobotException(TAG, "Normalized angle out of range " + angle);               break;
            }

            case "SLEEP": {
                int sleepValue = commandXPath.getInt("ms");
                sleep(sleepValue);
                break;
            }

            default: {
                throw new AutonomousRobotException(TAG, "No support for the command " + commandName);
            }
        }
    }

}


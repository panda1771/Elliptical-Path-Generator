package org.firstinspires.ftc.teamcode.auto;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.auto.vision.ImageProvider;
import org.firstinspires.ftc.teamcode.auto.vision.VuforiaImage;
import org.firstinspires.ftc.teamcode.auto.vision.VuforiaWebcamClose;
import org.firstinspires.ftc.teamcode.auto.vision.VuforiaWebcamInit;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXMLFreightFrenzy;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.motion.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.motion.drive.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.robot.motion.drive.DriveTrainMotion;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.Date;
import java.util.List;
import java.util.logging.Level;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;
import javax.xml.xpath.XPathExpressionException;

public class FTCAuto {

    private static final String TAG = "FTCAuto";

    private final LinearOpMode linearOpMode;
    private final FTCRobot robot;
    private final DriveTrainMotion motion;
    private double desiredHeading = 0.0; // always normalized

    private final RobotConstants.Alliance alliance;
    private final RobotConstantsFreightFrenzy.OpMode autoOpMode;
    private final String workingDirectory;

    private final RobotActionXMLFreightFrenzy.RobotActionDataFreightFrenzy actionData; // for the selected OpMode

    // Webcam and Vumarks.
    private VuforiaWebcamInit vuforiaWebcam;
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaWebcamClose vuforiaWebcamClose;

    // Load OpenCV.
    private static boolean openCVInitialized = false;

    static {
        // Android only
        if (OpenCVLoader.initDebug())
            openCVInitialized = true;
    }

    // Main class for the autonomous run.
    public FTCAuto(RobotConstantsFreightFrenzy.OpMode pAutoOpMode, RobotConstants.Alliance pAlliance, LinearOpMode pLinearOpMode)
            throws ParserConfigurationException, SAXException, XPathException, IOException, InterruptedException {

        RobotLogCommon.c(TAG, "FTCAuto constructor");

        // A failure in OpenCV initialization will prevent us from recognizing
        // the barcode.
        if (!openCVInitialized)
            throw new AutonomousRobotException(TAG, "Error in OpenCV initialization");

        autoOpMode = pAutoOpMode;
        alliance = pAlliance;
        if (alliance == RobotConstants.Alliance.UNKNOWN)
            throw new AutonomousRobotException(TAG, "Alliance is UNKNOWN");

        workingDirectory = WorkingDirectory.getWorkingDirectory();

        // Get the directory for the various configuration files.
        String xmlDirectory = workingDirectory + RobotConstants.xmlDir;

        // Read the robot action file for all opmodes. Extract data from
        // the parsed XML file for the selected OpMode only.
        RobotActionXMLFreightFrenzy actionXML = new RobotActionXMLFreightFrenzy(xmlDirectory);
        actionData = actionXML.getOpModeData(autoOpMode.toString());

        Level lowestLoggingLevel = actionData.lowestLoggingLevel;
        if (lowestLoggingLevel != null) // null means use the default
            RobotLogCommon.setMinimimLoggingLevel(lowestLoggingLevel);
        RobotLogCommon.c(TAG, "Lowest logging level " + RobotLogCommon.getMinimumLoggingLevel());

        // Initialize the hardware and methods the control motion.
        linearOpMode = pLinearOpMode;
        robot = new FTCRobot(linearOpMode);
        motion = new DriveTrainMotion(linearOpMode, robot);

        // Start the asynchronous initialization of Vuforia.
        if (robot.webcam1Name != null) {
            RobotLogCommon.d(TAG, "Vuforia: start asynchronous initialization");
            vuforiaWebcam = new VuforiaWebcamInit(robot.webcam1Name);
        }

        // Other initialization code goes here.

        // Wait for the asynchronous initialization of Vuforia to complete.
        if (robot.webcam1Name != null) {
            RobotLogCommon.d(TAG, "wait for initialization");
            vuforiaLocalizer = vuforiaWebcam.waitForVuforiaInitialization();
        }

        RobotLogCommon.c(TAG, "FTCAuto construction complete");
    }

    public void runRobot() throws XPathException, InterruptedException, IOException {

        RobotLogCommon.i(TAG, "At start");


        // Safety check against ftc runtime initialization errors.
        // Make sure the opmode is still active.
        if (!linearOpMode.opModeIsActive())
            throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runRobot()");

        // Follow the choreography specified in the robot action file.
        List<RobotXMLElement> actions = actionData.actions;
        try {
            for (RobotXMLElement action : actions) {

                if (!linearOpMode.opModeIsActive())
                    return; // better to just bail out

                doCommand(action); // no, but doCommand may change that
            }
        } finally {
            robot.imu.stopIMUReader();

            //## It's better not to try to shut down Vuforia - it may be
            // time-consuming. See comments at "CLOSE_WEBCAM".

            RobotLogCommon.i(TAG, "Exiting FTCAuto");
            linearOpMode.telemetry.addData("FTCAuto", "COMPLETE");
            linearOpMode.telemetry.update();
        }
    }

    //===============================================================================================
    //===============================================================================================

    // Using the XML elements and attributes from the configuration file RobotAction.xml,
    // execute the action.
    private void doCommand(RobotXMLElement pAction) throws InterruptedException, XPathException {

        // Set up XPath access to the current action command.
        XPathAccess commandXPath = new XPathAccess(pAction);
        String commandName = pAction.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing FTCAuto command " + commandName);

        //!! If a command is needed in TeleOp do not simply copy/paste the code
        // into RobotActionTeleOp. Create a static method that can be accessed
        // by FTCAuto and RobotActionTeleOp. Examples are straight_by() and
        // turn().
        switch (commandName) {

            // The robot moves without rotation in a direction relative to
            // the robot's current heading according to the "angle" parameter.
            // Just as lineTo and strafeTo are synonymous in Road Runner
            // make STRAIGHT_BY and STRAFE_BY synonymous here. But the
            // use of STRAFE_BY is clearest when the angle is not divisible
            // by 90. When it is, use the specializations STRAFE_RIGHT and
            // STRAFE_LEFT below.

            // or is RUN_TO_POSITION good enough?
            case "STRAIGHT_BY": {
                double angle = commandXPath.getDouble("angle");
                straight_by(commandXPath, robot.driveTrain, motion, angle, desiredHeading);
                break;
            }

            // Specialization of STRAIGHT_BY.
            case "FORWARD": {
                straight_by(commandXPath, robot.driveTrain, motion, 0.0, desiredHeading);
                break;
            }

            // Specialization of STRAIGHT_BY.
            case "BACK": {
                straight_by(commandXPath, robot.driveTrain, motion, -180.0, desiredHeading);
                break;
            }

            // Specialization of STRAIGHT_BY.
            case "STRAFE_LEFT": {
                straight_by(commandXPath, robot.driveTrain, motion, 90.0, desiredHeading);
                break;
            }

            // Specialization of STRAIGHT_BY.
            case "STRAFE_RIGHT": {
                straight_by(commandXPath, robot.driveTrain, motion, -90.0, desiredHeading);
                break;
            }

            // A normalized turn, i.e. a turn from 0 to +180 (not inclusive)
            // degrees CCW and 0 to -180 degrees CW, which will always be the
            // shortest distance from the current heading to the desired heading.
            // Allow 0 angle for deskew, i.e. turn to desired heading.
            case "TURN": {
                double angle = commandXPath.getDouble("angle");
                if ((angle >= 0 && angle < 180) || (angle <= 0 && angle >= -180))
                    desiredHeading = turn(commandXPath, motion, desiredHeading, angle, DriveTrainConstants.TurnType.NORMALIZED);
                else
                    throw new AutonomousRobotException(TAG, "Normalized angle out of range " + angle);
                break;
            }

            // Turn in the requested direction without FTC normalization.
            // The robot may not take the shortest path to the target heading.
            // Allow 0 angle for deskew, i.e. turn to desired heading.
            case "TURN_UNNORMALIZED": {
                double angle = commandXPath.getDouble("angle");
                if ((angle >= 0 && angle < 360) || (angle <= 0 && angle > -360))
                    desiredHeading = turn(commandXPath, motion, desiredHeading, angle, DriveTrainConstants.TurnType.UNNORMALIZED);
                else
                    throw new AutonomousRobotException(TAG, "Unnormalized angle out of range " + angle);
                break;
            }

            // For testing, take a picture and write it out to a file.
            case "TAKE_PICTURE": {
                if (vuforiaLocalizer == null || vuforiaWebcamClose != null)
                    throw new AutonomousRobotException(TAG, "Vuforia not initialized or shutting down");

                ImageProvider imageProvider = new VuforiaImage(vuforiaLocalizer);
                Pair<Mat, Date> image = imageProvider.getImage();
                if (image.first == null) {
                    RobotLogCommon.d(TAG, "Unable to get image from the camera");
                    linearOpMode.telemetry.addData("Take picture:", "unable to get image from the camera");
                    linearOpMode.telemetry.update();
                    return;
                }

                RobotLogCommon.d(TAG, "Took a picture");
                String fileDate = TimeStamp.getDateTimeStamp(image.second);
                String outputFilenamePreamble = workingDirectory + RobotConstants.imageDir + "Image_" + fileDate;

                // The image may be RGB (from a camera) or BGR ( OpenCV imread from a file).
                Mat imgOriginal = image.first.clone();

                // If you don't convert RGB to BGR here then the _IMG.png file will be written
                // out with incorrect colors (gold will show up as blue).
                if (imageProvider.getImageFormat() == ImageProvider.ImageFormat.RGB)
                    Imgproc.cvtColor(imgOriginal, imgOriginal, Imgproc.COLOR_RGB2BGR);

                String imageFilename = outputFilenamePreamble + "_IMG.png";
                RobotLogCommon.d(TAG, "Writing image " + imageFilename);
                Imgcodecs.imwrite(imageFilename, imgOriginal);

                RobotLogCommon.d(TAG, "Image width " + imgOriginal.cols() + ", height " + imgOriginal.rows());
                linearOpMode.telemetry.addData("Take picture:", "successful");
                linearOpMode.telemetry.update();
                break;
            }

            // In testing on 6/29/21 we noticed, and the logs showed, that
            // Vuforia shutdown took 5 seconds. This doesn't happen every time
            // but it's a huge amount of time -- so shut down in a separate
            // thread.
            //## Note: this will work at the beginning of an autonomous run
            // but is unwise near the end of the run because you may risk
            // getting "stuck on stop()". near the end of a run it's better
            // not to close the camera.
            case "CLOSE_WEBCAM": {
                if (vuforiaLocalizer != null && vuforiaWebcamClose == null) {
                    RobotLogCommon.i(TAG, "Shutting down Vuforia");
                    vuforiaWebcamClose = new VuforiaWebcamClose(vuforiaLocalizer);
                }
                break;
            }

            case "SLEEP": { //I want sleep :)
                int sleepValue = commandXPath.getInt("ms");
                RobotLogCommon.d(TAG, "Pause by " + sleepValue + " milliseconds");
                sleep(sleepValue);
                break;
            }

            case "BREAKPOINT": {
                while (!linearOpMode.gamepad1.a) {
                    sleep(1);
                }
                break;
            }

            default: {
                throw new AutonomousRobotException(TAG, "No support for the command " + commandName);
            }
        }
    }

    // Make these methods public static so that they can be accessed in TeleOp.
    public static void straight_by(XPathAccess pCommandXPath, DriveTrain pDriveTrain, DriveTrainMotion pMotion, double pAngle, double pDesiredHeading) throws XPathExpressionException {

        // Start with the absolute value of the click count. The sign of the click count for each of the
        // motors will be adjusted depending on the sign of the angle.
        int targetClicks = (int) Math.abs(pCommandXPath.getDouble("distance") * pDriveTrain.getClicksPerInch());

        // Default to no ramp-down.
        double rampDownInches = Math.abs(pCommandXPath.getDouble("ramp_down_at_inches_remaining", 0.0));
        int rampDownAtClicksRemaining = (int) (rampDownInches * pDriveTrain.getClicksPerInch());

        // Sanity check.
        if (rampDownAtClicksRemaining > targetClicks)
            rampDownAtClicksRemaining = targetClicks;

        // The velocity factor must be positive and in the range > 0.0 and <= 1.0
        double velocity = Math.abs(pCommandXPath.getDouble("velocity")); // fraction of maximum
        if (velocity <= 0.0 || velocity > 1.0 || velocity < DriveTrainConstants.MINIMUM_DOMINANT_MOTOR_VELOCITY)
            throw new AutonomousRobotException(TAG, "velocity out of range " + velocity);

        pMotion.straight(targetClicks, pAngle, velocity, rampDownAtClicksRemaining, pDesiredHeading);
    }

    public static double turn(XPathAccess pCommandXPath, DriveTrainMotion pMotion, double pDesiredHeading, double pAngle, DriveTrainConstants.TurnType pTurnType) throws XPathExpressionException {
        double power = Math.abs(pCommandXPath.getDouble("power")); // fraction of maximum
        if (power > 1.0 || power < DriveTrainConstants.MINIMUM_TURN_POWER)
            throw new AutonomousRobotException(TAG, "Power out of range " + power);

        // Default to no ramp-down.
        double rampDownAtDegreesRemaining = pCommandXPath.getDouble("ramp_down_at_degrees_remaining", 0.0);

        //Make the sign of the angle remaining be the same as the tturn angle.
        if (Math.signum(pAngle) != Math.signum(rampDownAtDegreesRemaining))
            throw new AutonomousRobotException(TAG, "Wrong sign for Angle Remaining.");

        // Sanity check.
        if (Math.abs(rampDownAtDegreesRemaining) > Math.abs(pAngle))
                rampDownAtDegreesRemaining = pAngle;

        return pMotion.turn(pDesiredHeading, pAngle, power, Math.abs(rampDownAtDegreesRemaining), pTurnType); // abs for consistency
    }
}


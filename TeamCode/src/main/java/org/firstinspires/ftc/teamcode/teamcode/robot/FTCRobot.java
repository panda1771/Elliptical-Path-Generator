package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.RobotConstants;

import org.firstinspires.ftc.teamcode.robot.imu.OptimizedIMU;
import org.firstinspires.ftc.teamcode.robot.motion.drive.DriveTrain;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.List;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

public class FTCRobot {

    private static final String TAG = "FTCRobotCore";

    public final LinearOpMode linearOpMode;
    public final HardwareMap hardwareMap;

    public DriveTrain driveTrain;
    public OptimizedIMU imu;
    public WebcamName webcam1Name;

    public RobotConfigXML configXML;

    public FTCRobot(LinearOpMode pLinearOpMode) throws InterruptedException {
        linearOpMode = pLinearOpMode;
        hardwareMap = linearOpMode.hardwareMap;

        // From the FTC example ConceptMotorBulkRead.java
        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors. [See DriveTrainCore.java]
        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        // Important Step 3: Set all Expansion hubs to use the AUTO Bulk Caching mode.
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        String workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.xmlDir;

        // Get configuration parameters from RobotConfig.xml.
        try {
            configXML = new RobotConfigXML(xmlDirectory);
            XPathAccess configXPath;

            // Get the current drive train configuration.
            configXPath = configXML.getPath("DRIVE_TRAIN");
            driveTrain = new DriveTrain(hardwareMap, configXPath);

            // Initialize the IMU.
            imu = new OptimizedIMU(linearOpMode);

            // See if configXML contains an entry for a webcam.
            configXPath = configXML.getPath("WEBCAM");
            String webcamInConfiguration = configXPath.getString("@configured", "yes");
            if (webcamInConfiguration.equals("yes"))
                webcam1Name = this.hardwareMap.get(WebcamName.class, "Webcam 1");
            // if not, leave webcam1Name null and test in FTCAuto

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (XPathExpressionException xpex) {
            throw new AutonomousRobotException(TAG, "Xpath Expression Exception " + xpex.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        }
    }

}


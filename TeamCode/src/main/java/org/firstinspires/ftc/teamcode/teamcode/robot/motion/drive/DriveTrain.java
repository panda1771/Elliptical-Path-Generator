package org.firstinspires.ftc.teamcode.robot.motion.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.motion.MotorCore;

import java.util.EnumMap;
import javax.xml.xpath.XPathExpressionException;

// Robot Drive Train
public class DriveTrain extends MotorCore {

    public static final String TAG = "DriveTrain";

    private final double clicksPerInch;

    public DriveTrain(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pConfigXPath);

        // Get the drive train configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining the robot's drive train");
        RobotLogCommon.c(TAG, "Motor(s) " + pConfigXPath.getString("motors/@model", true));

        // From the FTC example ConceptMotorBulkRead.java
        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        //## Element names match the robot configuration on the Driver Station
        DcMotorEx lf = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getElementText("motors/left_front_drive"));
        DcMotorEx rf = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getElementText("motors/right_front_drive"));
        DcMotorEx lb = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getElementText("motors/left_back_drive"));
        DcMotorEx rb = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getElementText("motors/right_back_drive"));

        // Set the direction of each motor.
        lf.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getString("motors/left_front_drive/@direction", true)));
        rf.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getString("motors/right_front_drive/@direction", true)));
        lb.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getString("motors/left_back_drive/@direction", true)));
        rb.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getString("motors/right_back_drive/@direction", true)));

        EnumMap<MotorId, DcMotorEx> motorMap = new EnumMap<MotorId, DcMotorEx>(MotorId.class) {{
            put(MotorId.LEFT_FRONT_DRIVE, lf);
            put(MotorId.RIGHT_FRONT_DRIVE, rf);
            put(MotorId.LEFT_BACK_DRIVE, lb);
            put(MotorId.RIGHT_BACK_DRIVE, rb);
        }};

        setMotors(motorMap);

        double wheelDiameterIn = pConfigXPath.getDouble("wheel_diameter_in");
        clicksPerInch = getClicksPerMotorRev() / (wheelDiameterIn * Math.PI);
    }

    public double getClicksPerInch() {
        return clicksPerInch;
    }

    // Assumes all clipping and all final modifications to the velocity,
    // e.g. running at .5 velocity, have already been performed.
    public void driveAllByVelocity(double lfVelocity, double rfVelocity,
                                   double lbVelocity, double rbVelocity) {
        EnumMap<MotorId, Double> velocityMap = new EnumMap<>(MotorId.class);
        velocityMap.put(MotorId.LEFT_FRONT_DRIVE, lfVelocity);
        velocityMap.put(MotorId.RIGHT_FRONT_DRIVE, rfVelocity);
        velocityMap.put(MotorId.LEFT_BACK_DRIVE, lbVelocity);
        velocityMap.put(MotorId.RIGHT_BACK_DRIVE, rbVelocity);

        driveAllByVelocity(velocityMap);
    }

}
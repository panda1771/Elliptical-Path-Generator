package org.firstinspires.ftc.teamcode.robot.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;

import java.io.IOException;
import java.util.List;

public class OptimizedIMU {

    private static final String TAG = "OptimizedIMUCore";
    private final LinearOpMode linearOpMode;
    private final IMUReader imuReader;

    public OptimizedIMU(LinearOpMode pLinearOpMode) throws InterruptedException {
        linearOpMode = pLinearOpMode;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        //      parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        //      parameters.loggingEnabled = true;
        //      parameters.loggingTag = "IMU";

        // Use IMU optimization.
        //## reference https://github.com/acmerobotics/relic-recovery/blob/master/TeamCode/src/main/java/com/acmerobotics/relicrecovery/opmodes/test/ExpansionHubBenchmark.java
        //## Tests in July 2021 show that the IMU optimization via the LynxEmbeddedIMU
        // actually makes a difference: 100 to 110 reads per second unoptimized vs
        // about 140 reads per second optimized.

        // All Lynx modules have been set to LynxModule.BulkCachingMode.AUTO
        // in FTCRobotCore.java.
        BNO055IMU imu = null;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : modules) {
            // Both the Control Hub and the Expansion Hub have an IMU. Just use the one
            // from the Control Hub, which is the parent.
            if (module.isParent()) {
                RobotLogCommon.d(TAG, "Connecting to the IMU on the Control Hub: module = " + module.getModuleAddress());
                imu = new LynxEmbeddedIMU(OptimizedI2cDevice.createLynxI2cDeviceSynch(module, 0));
                imu.initialize(parameters);
                break;
            }
        }

        if (imu == null)
            throw new AutonomousRobotException(TAG, "Could not connect to the IMU on the Control Hub");

        imuReader = new IMUReader(linearOpMode, imu);
        imuReader.startIMUReader();
    }

    //?? calibration - see the sample SensorBNO055IMUCalibration and the comments --
    // "The BNO055 is internally self-calibrating and thus can be very successfully
    // * used without manual intervention. That said, performing a one-time calibration, saving the
    // * results persistently, then loading them again at each run can help reduce the time that automatic
    // * calibration requires

    public double getIMUHeading() {
        return imuReader.getIMUHeading();
    }

    public double getIMUPitch() {
        return imuReader.getIMUPitch();
    }

    public double getIMUReadRate() { return imuReader.getIMUReadRate(); }

    public void stopIMUReader() throws IOException, InterruptedException {
        imuReader.stopIMUReader();
    }

}
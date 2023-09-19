package org.firstinspires.ftc.teamcode.robot.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutoWorker;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.IOException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

// Class that continuously reads IMU data and posts the results.
//## Not designed as a Singleton and not intended that an instance
//## of this class be shared between threads.
public class IMUReader {

    private static final String TAG = "IMUReader";

    private final LinearOpMode linearOpMode;
    private final BNO055IMU imu;
    private boolean imuActivated = false;

    // Thread-related.
    private CountDownLatch countDownLatch;
    private IMUReaderCallable imuReaderCallable;
    private CompletableFuture<Void> imuReaderFuture;

    private final AtomicReference<Orientation> imuOrientation;
    private final ElapsedTime imuTimer;
    private final AtomicInteger imuReadCount;

    // The IMU must have been previously initialized.
    public IMUReader(LinearOpMode pLinearOpMode, BNO055IMU pInitializedIMU) {
        linearOpMode = pLinearOpMode;
        imu = pInitializedIMU;
        imuOrientation = new AtomicReference<>(imu.getAngularOrientation().toAxesReference(INTRINSIC).toAxesOrder(ZYX));
        imuTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        imuReadCount = new AtomicInteger();
    }

    // Start the IMU reader thread.
    public void startIMUReader() throws InterruptedException {
        if (imuActivated)
            return; // nothing to do
        imuActivated = true;

        // Start up the IMU reader as a CompletableFuture.
        RobotLogCommon.i(TAG, "Starting IMU reader thread");

        countDownLatch = new CountDownLatch(1);
        imuReaderCallable = new IMUReaderCallable();
        imuReaderFuture = Threading.launchAsync(imuReaderCallable);

        // Wait here until the thread is off and running.
        countDownLatch.await();
        RobotLogCommon.i(TAG, "Wait for CountDownLatch done; IMU reader thread is running");
    }

    // Turn off when done with the IMU.
    public void stopIMUReader() throws IOException, InterruptedException {
        if (!imuActivated)
            return; // nothing to do
        imuActivated = false;

        // Stop the reader and wait for it to complete.
        imuReaderCallable.stopThread();
        Threading.getFutureCompletion(imuReaderFuture);
    }

    // Returns the number of imu reads per second.
    public double getIMUReadRate() {
        int myReadCount = imuReadCount.get();
        double numSec = imuTimer.time() / 1000;
        return (numSec != 0) ? myReadCount / numSec : 0.0;
    }

    public double getIMUHeading() {
        Orientation angles = imuOrientation.get();
        return (DEGREES.normalize(angles.firstAngle));
    }

    //# NOTE: the IMU on our 2018-2019 robot is oriented such that "secondAngle"
    //  reports pitch and "thirdAngle" reports roll. This is unlike the
    //  SensorBNO055IMU sample:
    //           .addData("roll", new Func<String>() {
    //                @Override public String value() {
    //                    return formatAngle(angles.angleUnit, angles.secondAngle);
    public double getIMUPitch() {
        Orientation angles = imuOrientation.get();
        return (DEGREES.normalize(angles.secondAngle));
    }

    // Reads the IMU and makes its data available to the main thread.
    private class IMUReaderCallable extends AutoWorker<Void> {

        IMUReaderCallable() {
            super();
        }

        public Void call() {
            RobotLogCommon.i(TAG, "In IMU thread");
            countDownLatch.countDown(); // signal that I'm running
            imuTimer.reset(); // start timing
            imuReadCount.set(0);

            //## the linearOpMode is not active at this point so do not test.
            while (!stopThreadRequested()) {
                imuOrientation.set(imu.getAngularOrientation().toAxesReference(INTRINSIC).toAxesOrder(ZYX));
                imuReadCount.incrementAndGet();
            }

            return null;
        }
    }

}

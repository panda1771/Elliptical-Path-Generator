package org.firstinspires.ftc.teamcode.robot.motion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;

import java.util.EnumMap;
import java.util.Objects;

import javax.xml.xpath.XPathExpressionException;

// Base class for all DC motors. Descendants of this class may define a single
// motor, such as a shooter, or multiple motors, such as the four motors of the
// robot's drive train. All motors in a single instance of this class must be
// of the same type such as NeveRest 20.
public class MotorCore {

    private static final String TAG = "MotorCore";

    // All motors on the robot.
    public enum MotorId {
        LEFT_FRONT_DRIVE, RIGHT_FRONT_DRIVE, LEFT_BACK_DRIVE, RIGHT_BACK_DRIVE,
        ELEVATOR_LEFT, ELEVATOR_RIGHT, FREIGHT_DELIVERY
    }

    private final double clicksPerMotorRev;
    private final double maxVelocity; // clicks per second

    private EnumMap<MotorId, DcMotorEx> motorMap;

    public MotorCore(XPathAccess pConfigXPath) throws XPathExpressionException {
        clicksPerMotorRev = pConfigXPath.getDouble("clicks_per_motor_rev");
        double motorRPM = pConfigXPath.getDouble("rpm");
        maxVelocity = Math.floor((clicksPerMotorRev * motorRPM) / 60); // clicks per second
    }

    protected void setMotors(EnumMap<MotorId, DcMotorEx> pMotorMap) {
       motorMap = pMotorMap;
       
        // Set default run mode.
        /*
            From https://docs.revrobotics.com/rev-control-system/programming/using-encoder-feedback
            In RUN_USING_ENCODER mode, you should set a velocity (measured in ticks per second),
            rather than a power level. You can still provide a power level in RUN_USING_ENCODER
            mode, but this is not recommended, as it will limit your target speed significantly.
            Setting a velocity from RUN_WITHOUT_ENCODER mode will automatically switch the motor
            to RUN_USING_ENCODER mode.
        */
        //## Note that with either of the run modes RUN_USING_ENCODER or RUN_TO_POSITION,
        // setPower has no effect!!
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBrakeAll();
    }

    public void setMode(MotorId pMotorId, DcMotor.RunMode pMode) {
       Objects.requireNonNull(motorMap.get(pMotorId)).setMode(pMode);
    }

    public void setModeAll(DcMotor.RunMode pMode) {
        motorMap.forEach((k, v) -> v.setMode(pMode));
    }

    private void setZeroPowerBrakeAll() {
        motorMap.forEach((k, v) -> v.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void setTargetPosition(MotorId pMotorId, int pTargetClicks) {
        Objects.requireNonNull(motorMap.get(pMotorId)).setTargetPosition(pTargetClicks);
    }

    public double getClicksPerMotorRev() {
        return clicksPerMotorRev;
    }

    public int getClickCount(MotorId pMotorId) {
        return Objects.requireNonNull(motorMap.get(pMotorId)).getCurrentPosition();
    }

    // Assumes all clipping and all final modifications to the velocity,
    // e.g. running at .5 velocity, have already been performed.
    //**TODO exception if any motor is in RUN_WITHOUT_ENCODER mode.
    public void driveAllByVelocity(EnumMap<MotorId, Double> pVelocityMap) {
        pVelocityMap.forEach((k, v) -> Objects.requireNonNull(motorMap.get(k)).setVelocity(v * maxVelocity));
    }

    // For use with DcMotor.RunMode.RUN_WITHOUT_ENCODER.
    // Assumes all clipping and all final modifications to the power,
    // e.g. running at .5 power, have already been performed.
    //**TODO exception if any motor is NOT in RUN_WITHOUT_ENCODER mode.
    public void driveAllByPower(EnumMap<MotorId, Double> pPowerMap) {
        pPowerMap.forEach((k, v) -> Objects.requireNonNull(motorMap.get(k)).setPower(v));
    }

    public boolean isBusy(MotorId pMotorId) {
        if (Objects.requireNonNull(motorMap.get(pMotorId)).getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            throw new AutonomousRobotException(TAG, "Illegal test of isBusy((); motor " + pMotorId + " has not been set to RUN_TO_POSITION");
        return Objects.requireNonNull(motorMap.get(pMotorId)).isBusy();
    }

    //**TODO exception if any motor is in RUN_WITHOUT_ENCODER mode.
    public void stopAllZeroVelocity() {
        motorMap.forEach((k, v) -> v.setVelocity(0.0));
    }

    //**TODO exception if any motor is NOT in RUN_WITHOUT_ENCODER mode.
    public void stopAllZeroPower() {
        motorMap.forEach((k, v) -> v.setPower(0.0));
    }

}

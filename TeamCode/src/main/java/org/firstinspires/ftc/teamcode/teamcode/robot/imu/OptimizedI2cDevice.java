package org.firstinspires.ftc.teamcode.robot.imu;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

// see https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/I2cDeviceSynchImplOnSimple.html
// Class I2cDeviceSynchImpl
// I2cDeviceSynchImpl(I2cDevice i2cDevice, boolean isI2cDeviceOwned
public class OptimizedI2cDevice extends I2cDeviceSynchImplOnSimple {
    public OptimizedI2cDevice(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
        super(simple, isSimpleOwned);
    }

    @Override
    public void setReadWindow(ReadWindow window) {
        // intentionally empty
    }

    public static I2cDeviceSynch createLynxI2cDeviceSynch(LynxModule module, int bus) {
        return new OptimizedI2cDevice( LynxFirmwareVersionManager.createLynxI2cDeviceSynch(AppUtil.getDefContext(), module, bus), true);
    }
}

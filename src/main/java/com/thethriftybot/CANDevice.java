/**
 * Copyright (C) 2024 The Thrifty Bot, LLC
 * License here...
 */
package com.thethriftybot;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.CANData;

public abstract class CANDevice {

    private final static int THRIFTY_VENDOR_ID = 13;

    // Ordinality matters
    public static enum DeviceType {
        BROADCAST_MSG,
        ROBOT_CTRL,
        MOTOR_CTRL,
        RELAY_CTRL,
        GYRO_SENSOR,
        ACCELEROMETER_SENSOR,
        ULTRASONIC_SENSOR,
        GEAR_TOOTH_SENSOR,
        POWER_DISTRIBUTION_MODULE,
        PNEUMATICS_CTRL,
        MISC,
        IO_BREAKOUT;
    }
    
    /**
     * API class ID calculation from STATUS/CONTROL state and index.
     */
    protected static enum APIClass {
        STATUS(20), CTRL(10);

        private final int code;
        private APIClass(int code) { this.code = code; }

        public int get(int index) {
            return ((code << 4) + index);
        }
    }

    protected int canID;
    protected int deviceHandle;

    /**
     * Create some CAN device
     * @param canID
     */
    protected CANDevice(int canID, DeviceType type) {
        this.canID = canID; 
        this.deviceHandle = CANAPIJNI.initializeCAN(THRIFTY_VENDOR_ID, canID,
            type.ordinal());
    }

    /**
     * Returns the CAN ID the device was initialized with.
     * @return The device's CAN ID.
     */
    public int getID() { return canID; }

    protected int reciveStatusFrame(int index, int startByte, int endByte) {
        final CANData dataBuffer = new CANData();
        CANAPIJNI.readCANPacketLatest(
            deviceHandle,
            APIClass.STATUS.get(index),
            dataBuffer
        );

        int motorData = 0;
        for (int i = startByte; i <= endByte; i++)
            motorData += (dataBuffer.data[i] & 0xFF) << ((endByte - i) * 8);
        return motorData;
    }
}

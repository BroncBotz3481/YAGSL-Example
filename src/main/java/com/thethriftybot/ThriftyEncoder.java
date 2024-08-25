/**
 * ThriftyAPI ThriftyEncoder
 * 
 * Copyright (C) 2024 The Thrifty Bot, LLC
 * 
 * License here...
 */
package com.thethriftybot;

import com.thethriftybot.Conversion.PositionUnit;
import com.thethriftybot.Conversion.VelocityUnit;

import edu.wpi.first.hal.CANAPIJNI;

/**
 * API for interfacing with the Thrifty CAN Encoder.
 * 
 * URL to docs here:
 * https://app.gitbook.com/o/jp5Nacgm5lihxKSYCOea/s/kwoaJVjf6wU0m0fEZUSF/software-resources/getting-started
 * 
 * @version 1.1.0
 */
public final class ThriftyEncoder extends CANDevice {

    /**
     * Creates a Thrifty CAN Encoder with the given CAN ID.
     * @param canID The CAN ID of the encoder.
     */
    public ThriftyEncoder(int canID) {
        super(canID, DeviceType.MISC);
    }

    public static enum Direction {
        CLOCKWISE, COUNTERCLOCKWISE;
    }

    // Set zero pos, config index 1
    public void debugConfigureCanID(int canID) {
        setConfig(0, canID & 0xFF);
    }

    /**
     * Configures the position which will report zero, tares the encoder.
     * @param unit The position unit to set the zero in.
     * @param zero The zero position in the given unit.
     */
    public ThriftyEncoder setZeroPosition(PositionUnit unit, double zero) {
        setConfig(1, (int)((zero / unit.value) * 0x3FFF));
        return this;
    }

    /**
     * Configures which direction of rotation yields positive readings.
     * @param direction Direction.CLOCKWISE or Direction.COUNTERCLOCKWISE.
     */
    public void setDirection(Direction direction) {
        setConfig(2, direction == Direction.CLOCKWISE ? 0 : 1);
    }
    
    // Set a config, status/index=1
    // |   1   ||   2   ||   3   ||   4   ||   5   ||   6   ||   7   ||   8   |
    // [ Value                            ][ Target                           ] 
    private void setConfig(int target, int value) {
        CANAPIJNI.writeCANPacketNoThrow(
            deviceHandle, 
            new byte[] {
                (byte)((value >> 24) & 0xFF), 
                (byte)((value >> 16) & 0xFF), 
                (byte)((value >> 8) & 0xFF), 
                (byte)(value & 0xFF),
                (byte)((target >> 24) & 0xFF), 
                (byte)((target >> 16) & 0xFF), 
                (byte)((target >> 8) & 0xFF), 
                (byte)(target & 0xFF)
            },
            APIClass.CTRL.get(1)
        );
    }

    // Status CAN Frame to Recive, ctrl/index=1
    // |   1   ||   2   ||   3   ||   4   ||   5   ||   6   ||   7   ||   8   |
    // [ Position       ][ Velocity       ]
    
    /**
     * Get the measured position in a given position unit.
     * @param unit The position unit to report measured values.
     * @return The encoder position reading.
     */
    public double getPosition(PositionUnit unit) {
        int raw = reciveStatusFrame(1, 0, 1);
        return ((double)raw / 0x3FFF) * unit.value;
    }
  
    /**
     * Get the measured velocity in a given  unit.
     * @param unit The velocity unit to report measured values.
     * @return The encoder velocity reading.
     */
    public double getVelocity(VelocityUnit unit) {
        int raw = reciveStatusFrame(1, 2, 3) * 1000;
        int adj = raw > 0x3FFF ? -(raw & 0x3FFF) : raw & 0x3FFF;
        return ((double)adj / 0x3FFF) * unit.value;
    }
}

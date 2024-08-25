// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swervelib.encoders;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.thethriftybot.ThriftyEncoder;
import com.thethriftybot.Conversion.PositionUnit;
import com.thethriftybot.Conversion.VelocityUnit;

/**
 * Swerve Absolute Encoder for Thrifty Encoders.
 */
public class ThriftyEncoderSwerve extends SwerveAbsoluteEncoder {
    /** The ThriftyEncoder */
    private ThriftyEncoder encoder;

    /**
     * Constructs a new ThriftyEncoderSwerve with the specified CAN ID.
     * @param canID
     */
    public ThriftyEncoderSwerve(int canID) {
        encoder = new ThriftyEncoder(canID);
    }

    /**
     * Sets the ThriftyEncoder to its factory default state.
     */
    @Override
    public void factoryDefault() {
    }

    /**
     * Clears any sticky faults associated with the ThriftyEncoder.
     */
    @Override
    public void clearStickyFaults() {
    }

    /**
     * Configures the object with the given inverted flag.
     *
     * @param inverted a boolean indicating whether the object should be inverted
     */
    @Override
    public void configure(boolean inverted) {
    }

    /**
     * Retrieves the absolute position of the encoder in degrees.
     *
     * @return the absolute position of the encoder in degrees
     */
    @Override
    public double getAbsolutePosition() {
        return encoder.getPosition(PositionUnit.DEGREES);
    }

    /**
     * Returns the absolute encoder object.
     *
     * @return the absolute encoder object
     */
    @Override
    public Object getAbsoluteEncoder() {
        return encoder;
    }

    /**
     * Sets the absolute encoder offset to the specified value.
     *
     * @param offset the new offset value in degrees
     * @return true if the offset was successfully set, false otherwise
     */
    @Override
    public boolean setAbsoluteEncoderOffset(double offset) {
        encoder.setZeroPosition(PositionUnit.DEGREES, offset);
        return true;
    }

    /**
     * Retrieves the velocity of the encoder in degrees per second.
     *
     * @return the velocity of the encoder in degrees per second
     */
    @Override
    public double getVelocity() {
        return encoder.getVelocity(VelocityUnit.DEGREES_PER_SEC);
    }

}

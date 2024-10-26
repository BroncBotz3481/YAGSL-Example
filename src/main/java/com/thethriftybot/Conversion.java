/**
 * Copyright (C) 2024 The Thrifty Bot, LLC
 * License here...
 */
package com.thethriftybot;

import com.thethriftybot.ThriftyNova.EncoderType;

public final class Conversion {
    private double ratio;

    public Conversion(PositionUnit units, EncoderType encoder) { this.ratio = encoder.ticks / units.value; }
    public Conversion(VelocityUnit units, EncoderType encoder) { this.ratio = encoder.ticks / units.value; }

    public double toMotor(double value) { return value * ratio; }
    public double fromMotor(double value) { return value / ratio; }

    public static enum PositionUnit {
        RADIANS(Math.PI * 2),
        DEGREES(360),
        ROTATIONS(1);

        public final double value;
        private PositionUnit(double value) { this.value = value; }
    }

    public static enum VelocityUnit {
        RADIANS_PER_SEC(Math.PI * 2),
        DEGREES_PER_SEC(360),
        ROTATIONS_PER_SEC(1),
        ROTATIONS_PER_MIN(1/60f);

        public final double value;
        private VelocityUnit(double value) { this.value = value; }
    }
}
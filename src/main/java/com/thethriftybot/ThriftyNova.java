/**
 * Copyright (C) 2024 The Thrifty Bot, LLC
 * 
 * ThriftyAPI ThriftyNova
 * 
 * License here...
 */
package com.thethriftybot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;

/**
 * API for interfacing with the Thrifty Nova motor controller.
 * 
 * URL to docs here:
 * https://app.gitbook.com/o/jp5Nacgm5lihxKSYCOea/s/kwoaJVjf6wU0m0fEZUSF/software-resources/getting-started
 * 
 * @version 1.1.0
 */
public final class ThriftyNova extends CANDevice {

    // =====================================================
    // CONSTANTS AND ENUMERATIONS
    // =====================================================

    /**
     * Different types of control modes.
     * @apiNote Ordinality matters!
     */
    private static enum ControlType {
        NONE,
        PERCENT_OUTPUT,
        POSITION_INTERNAL,
        VELOCITY_INTERNAL,
        POSITION_QUAD,
        VELOCITY_QUAD,
        POSITION_ABS;
        // POSITION_CAN,    (WIP)
        // VELOCITY_CAN     (WIP)
    }

    /**
     * Different encoder types.
     */
    public static enum EncoderType {
        INTERNAL(42, 1), QUAD(4096, 2), ABS(4096, 4);
        // CAN          (WIP)

        public final int ticks;
        public final int id;
        private EncoderType(int ticks, int id) { 
            this.ticks = ticks; 
            this.id = id;
        }
    }

    /**
     * Different types of current control.
     * @apiNote Ordinality matters!
     */
    public static enum CurrentType {
        STATOR, SUPPLY
    }

    /**
     * The various PID configuration slots.
     * @apiNote Ordinality matters!
     */
    public static enum PIDSlot {
        SLOT0, SLOT1;
    }

    /**
     * Different types of configuration errors.
     */
    public static enum Error {
        SET_INVERSION,
        SET_BREAK_MODE,
        SET_MAX_FWD, //forward
        SET_MAX_REV, //erse
        SET_RAMP_UP,
        SET_RAMP_DOWN,
        // Change current limit types
        SET_MAX_CURRENT,
        SET_CURRENT_TYPE,

        SET_KP_0, SET_KP_1,
        SET_KI_0, SET_KI_1,
        SET_KD_0, SET_KD_1,
        SET_KF_0, SET_KF_1,
        SET_IZONE_0, SET_IZONE_1,
        
        SET_FREQ_FAULT,
        SET_FREQ_SENSOR,
        SET_FREQ_QUAD_SENSOR,
        SET_FREQ_CTRL,
        SET_FREQ_CURRENT,

        SET_SOFT_LIMIT_FWD,
        SET_SOFT_LIMIT_REV,

        ENABLE_SOFT_LIMIT,
        ENABLE_HARD_LIMIT
    }

    // =====================================================
    // PID & CAN FREQUENCY SUB-CONFIGS
    // =====================================================

    private static abstract class SubConfig {
        final ThriftyNova motor;
        protected SubConfig(ThriftyNova motor) {
            this.motor = motor;
        }
    }

    /**
     * An internal PID controller configuration structure.
     */
    public static final class PIDConfig extends SubConfig {
        boolean slot0;

        private PIDConfig(ThriftyNova motor, int slot) { 
            super(motor); 
            this.slot0 = slot == 0;
        }

        /**
         * Set the PID controller terms from a PIDController object.
         * @param pid The PIDController object.
         * @return This motor instance.
         */
        public ThriftyNova set(PIDController pid) {
            setP(pid.getP());
            setI(pid.getI());
            setD(pid.getP());
            return motor;
        }

        /**
         * Sets the proportional term for the PID controller.
         * @param p The proportional term.
         * @return This motor instance.
         */
        public ThriftyNova setP(double p) {
            motor.setConfig(slot0 ? 10 : 14, translate(p), slot0 ? Error.SET_KP_0 : Error.SET_KP_1);
            return motor;
        }
        /**
         * Sets the integral term for the PID controller.
         * @param i The integral term.
         * @return This motor instance.
         */
        public ThriftyNova setI(double i) {
            motor.setConfig(slot0 ? 11 : 15, translate(i), slot0 ? Error.SET_KI_0 : Error.SET_KI_1);
            return motor;
        }
        /**
         * Sets the derivative term for the PID controller.
         * @param d The derivative term.
         * @return This motor instance.
         */
        public ThriftyNova setD(double d) {
            motor.setConfig(slot0 ? 12 : 16, translate(d), slot0 ? Error.SET_KD_0 : Error.SET_KD_1);
            return motor;
        }
        /**
         * Sets the feed forward term for the PID controller.
         * @param ff The feed forward term.
         * @return This motor instance.
         */
        public ThriftyNova setFF(double ff) {
            motor.setConfig(slot0 ? 13 : 17, translate(ff), slot0 ? Error.SET_KF_0 : Error.SET_KF_1);
            return motor;
        }

        private static int translate(double input) {
            return (int)(1000000 * input);
        }
    }

    /**
     * An internal CAN frame frequency configuration structure.
     */
    public static final class CANFreqConfig extends SubConfig {
        private CANFreqConfig(ThriftyNova motor) { 
            super(motor); 
        }

        /**
         * Set CAN frame frequency for faults.
         * @param per The period in seconds.
         * @return This motor instance.
         */
        public ThriftyNova setFault(double per) {
            motor.setConfig(18, translate(per), Error.SET_FREQ_FAULT);
            return motor;
        }
        /**
         * Set CAN frame frequency for encoder feedback.
         * @param per The period in seconds.
         * @return This motor instance.
         */
        public ThriftyNova setSensor(double per) {
            motor.setConfig(19, translate(per), Error.SET_FREQ_SENSOR);
            return motor;
        }
        /**
         * Set CAN frame frequency for quadrature encoder feedback.
         * @param per The period in seconds.
         * @return This motor instance.
         */
        public ThriftyNova setQuadSensor(double per) {
            motor.setConfig(20, translate(per), Error.SET_FREQ_QUAD_SENSOR);
            return motor;
        }
        /**
         * Set CAN frame frequency for control commands.
         * @param per The period in seconds.
         * @return This motor instance.
         */
        public ThriftyNova setControl(double per) {
            motor.setConfig(21, translate(per), Error.SET_FREQ_CTRL);
            return motor;
        }
        /**
         * Set CAN frame frequency for current measurements.
         * @param per The period in seconds.
         * @return This motor instance.
         */
        public ThriftyNova setCurrent(double per) {
            motor.setConfig(22, translate(per), Error.SET_FREQ_CURRENT);
            return motor;
        }
        private static int translate(double input) {
            return (int)(1000 * input);
        }
    }

    // =====================================================
    // CLASS FIELDS
    // =====================================================

    private double currentSetPoint = 0;

    public List<Error> errors;

    public final PIDConfig pid0, pid1;
    public final CANFreqConfig canFreq;

    private PIDSlot pidSlot = PIDSlot.SLOT0;
    private EncoderType encoderType = EncoderType.INTERNAL;
    private ControlType controlType = ControlType.PERCENT_OUTPUT;

    // =====================================================
    // CONSTRUCTORS
    // ===================================================== 

    /**
     * Instantiate a new ThriftyMotor object using a CAN identifier.
     * @param canID The CAN identifier for the particular device.
     */
    public ThriftyNova(int canID) {
        super(canID, DeviceType.MOTOR_CTRL);
        this.errors = new ArrayList<>();
        this.pid0 = new PIDConfig(this, 0);
        this.pid1 = new PIDConfig(this, 1);
        this.canFreq = new CANFreqConfig(this);
    }

    // =====================================================
    // ERROR HANDLING
    // =====================================================
    
    private void addError(Error err) {
        if (errors.size() < 256) 
            errors.add(err);
    }

    /**
     * Returns a list of the errors that have populated in the buffer.
     * @return A list of errors.
     */
    public List<Error> getErrors() { return errors; }

    /**
     * Clear the error buffer.
     */
    public void clearErrors() { errors.clear(); }

    /**
     * Checks if there is a specific error present in the buffer.
     * @param err The error to check for.
     * @return If the error exists or not.
     */
    public boolean hasError(Error err) {
        for (Error e : errors) 
            if (e == err) 
                return true;
        return false;
    }

    // =====================================================
    // CONFIGURATION FUNCTIONS
    // =====================================================

    /**
     * Set the inversion status of the motor.
     * @param inverted If the motor be inverted or not.
     * @return This motor instance.
     */
    public ThriftyNova setInverted(boolean inverted) {
        setConfig(0, inverted ? 1 : 0, Error.SET_INVERSION);
        return this;
    }

    /**
     * Set the break mode status of the motor.
     * @param brakeMode If the motor should be in brake mode or not.
     * @return This motor instance.
     */
    public ThriftyNova setBrakeMode(boolean brakeMode) {
        setConfig(1, brakeMode ? 1 : 0, Error.SET_BREAK_MODE);
        return this;
    }

    /**
     * Set the maximum percent output.
     * @param maxOutput Max forward [0, 1].
     * @return This motor instance.
     */
    public ThriftyNova setMaxOutput(double maxOutput) {
        setMaxOutput(maxOutput, maxOutput);
        return this;
    }

    /**
     * Set the maximum forward and reverse percent output.
     * @param maxFwd Max forward output [0, 1].
     * @param maxRev Max reverse output [0, 1].
     * @return This motor instance.
     */
    public ThriftyNova setMaxOutput(double maxFwd, double maxRev) {
        setConfig(3, translateOutput(limitRange(0, 1, maxFwd)), Error.SET_MAX_FWD);
        // this was originally             -limitRange(-1, 0, maxRev)   but I changed it
        setConfig(4, translateOutput(limitRange(0, 1, maxRev)), Error.SET_MAX_REV); 
        return this;
    }

    /**
     * Sets the time to ramp up in seconds from 0 to 10. For example, an 
     * input of 0.5 will ramp the motor from idle to 100% over the course 
     * of 0.5 seconds. 
     * @param rampUp The ramp up time.
     * @return This motor instance.
     */
    public ThriftyNova setRampUp(double rampUp) {
        rampUp = limitRange(0, 10, rampUp);

        double translatedRamp = 1;

        if(rampUp != 0) translatedRamp = 1 / (rampUp * 1000);    

        setConfig(5, translateOutput(translatedRamp), Error.SET_RAMP_UP);

        return this;
    }

    /**
     * Sets the time to ramp down in seconds from 0 to 10. For example, an 
     * input of 0.5 will ramp the motor from 100% to idle over the course
     * of 0.5 seconds. 
     * @param rampDown The ramp up time.
     * @return This motor instance.
     */
    public ThriftyNova setRampDown(double rampDown) 
    {
        rampDown = limitRange(0, 10, rampDown);

        double translatedRamp = 1;

        if(rampDown != 0) translatedRamp = 1 / (rampDown * 1000);   

        setMotorConfig(6, translateOutput(translatedRamp));

        return this;
    }

    /**
     * Set the max current the motor can draw in amps. Motor speed will be 
     * capped to satisfy the max current. Also set the what current reading
     * is used for limiting calculations, between:
     * - Stator: Uses the total draw of phase a, b, and c.
     * - Supply: Uses stator plus the draw of the controller itself.
     * @param currentType The current reading used for limiting calculations.
     * @param maxCurrent The max current.
     * @return This motor instance.
     */
    public ThriftyNova setMaxCurrent(CurrentType currentType, double maxCurrent) {
        setConfig(7, CANFreqConfig.translate(maxCurrent), Error.SET_MAX_CURRENT);
        setConfig(8, currentType.ordinal(), Error.SET_CURRENT_TYPE);
        return this;
    }

     /**
     * Sets the soft limits that the motor will not cross if soft limits are enabled.
     * @param revLimit The reverse position the motor will not go past.
     * @param fwdLimit The forward position the motor will not go past.
     * @return This motor instance.
     */
    public ThriftyNova setSoftLimits(double revLimit, double fwdLimit) {
        setConfig(26, (int)revLimit, Error.SET_SOFT_LIMIT_REV);
        setConfig(25, (int)fwdLimit, Error.SET_SOFT_LIMIT_FWD);
        return this;
    }

    /**
     * Enable / disable soft limits.
     * @param enable If soft limits should be enabled.
     * @return This motor instance.
     */
    public ThriftyNova enableSoftLimits(boolean enable) {
        setConfig(24, enable ? 1 : 0, Error.ENABLE_SOFT_LIMIT);
        return this;
    }

    /**
     * Enable / disable hard limits.
     * @param enable If hard limits should be enabled.
     * @return This motor instance.
     */
    public ThriftyNova enableHardLimits(boolean enable) {
        setConfig(23, enable ? 1 : 0, Error.ENABLE_HARD_LIMIT);
        return this;
    }

    /**
     * Set the encoder type to use for feedback control.
     * @param encoderType The encoder type to use.
     * @return This motor instance.
     */
    public ThriftyNova useEncoderType(EncoderType encoderType) {
        this.encoderType = encoderType;
        return this;
    }

    /**
     * Set the PID slot to use for feedback control.
     * @param pidSlot The PID slot to use.
     * @return This motor instance.
     */
    public ThriftyNova usePIDSlot(PIDSlot pidSlot) {
        this.pidSlot = pidSlot;
        return this;
    }
    
    // =====================================================
    // CONTROL SETTER FUNCTIONS
    // =====================================================

    /**
     * Sets the percent output of the motor from between 1.0 (full forward)
     * and -1.0 (full reverse), where 0.0 is full stop.
     * @param percentOutput The percent output of the motor from [-1, 1].
     */
    public void setPercent(double percentOutput) {
        currentSetPoint = 0;
        setMotorControl(ControlType.PERCENT_OUTPUT, translateOutput(limitRange(-1, 1, percentOutput)));
    }

    /**
     * Drives the motor towards the given position using the configured PID controller.
     * @param targetPosition The position to target.
     */
    public void setPosition(double targetPosition) {
        currentSetPoint = targetPosition;

        if (encoderType == EncoderType.INTERNAL) 
            setMotorControl(ControlType.POSITION_INTERNAL, (int)targetPosition);
        else if (encoderType == EncoderType.QUAD)
            setMotorControl(ControlType.POSITION_QUAD, (int)targetPosition);
        else if (encoderType == EncoderType.ABS)
            setMotorControl(ControlType.POSITION_ABS, (int)targetPosition);
        else {
            // Cannot set position with given encoder type (none rn)
        }
    }

    /**
     * Drives the motor at the given velocity using the configured PID controller.
     * @param targetVelocity The velocity to target.
     */
    public void setVelocity(double targetVelocity) {
        currentSetPoint = targetVelocity;

        if (encoderType == EncoderType.INTERNAL) 
            setMotorControl(ControlType.VELOCITY_INTERNAL, (int)targetVelocity);
        else if (encoderType == EncoderType.QUAD)
            setMotorControl(ControlType.VELOCITY_QUAD, (int)targetVelocity);
        else {
            // Cannot set velocity with the given encoder type (ex. abs)
        }
    }

    // =====================================================
    // STATUS & FEEDBACK GETTERS
    // =====================================================

    /**
     * Gets the encoder position measurement.
     * @return The position measurement.
     */
    public double getPosition() {
        return reciveStatusFrame(encoderType.id, 4, 7);
    }

    /**
     * Gets the encoder velocity measurement.
     * @return The velocity measurement.
     */
    public double getVelocity() {
        if (encoderType == EncoderType.ABS) return 0.0; // Cannot get velocity for an abs encoder
        return reciveStatusFrame(encoderType.id, 0, 3);
    }

    /**
     * Gets the stator current measurement.
     * @return The stator current measurement.
     */
    public int getStatorCurrent() {
        return reciveStatusFrame(4, 0, 1);
    }

    /**
     * Gets the supply current measurement.
     * @return The supply current measurement.
     */
    public int getSupplyCurrent() {
        return reciveStatusFrame(4, 2, 3);
    }

    /**
     * Returns the last set point specified by the user in any
     * control method. Percent output set point becomes zero.
     * @return The current set point.
     */
    public double getSetPoint() { return currentSetPoint; }


    /**
     * Returns the error to the last set point that was input,
     * based on last used control mode (position or velocity), 
     * and encoder type (internal or quaderature).
     * @return The closed loop error.
     */
    public double getClosedLoopError() {
        switch (controlType) {
            case POSITION_INTERNAL:
            case POSITION_QUAD:
            case POSITION_ABS:
                return currentSetPoint - getPosition();
            case VELOCITY_INTERNAL:
            case VELOCITY_QUAD:
                return currentSetPoint - getVelocity();
            default:
                return 0;
        }
    }

    /**
     * Sets the encoder position.
     * @param position The position to set the encoder to.
     */
    public void setEncoderPosition(double position) {
        CANAPIJNI.writeCANPacketNoThrow(
            deviceHandle, 
            new byte[] {
                (byte)(((int)position >> 24) & 0xFF), 
                (byte)(((int)position >> 16) & 0xFF), 
                (byte)(((int)position >> 8) & 0xFF), 
                (byte)((int)position & 0xFF),
                (byte)(encoderType.ordinal()), 
                (byte)(0), 
                (byte)(0), 
                (byte)(0)
            },
            APIClass.CTRL.get(2)
        );
    }

    // =====================================================
    // INTERNAL FUNCTIONS
    // =====================================================

    private void setMotorControl(ControlType ctrlType, int target) {
        controlType = ctrlType;

        CANAPIJNI.writeCANPacketNoThrow(
            deviceHandle, 
            new byte[] {
                0, 
                0, 
                (byte)pidSlot.ordinal(), 
                (byte)ctrlType.ordinal(), 
                (byte)((target >> 24) & 0xFF), 
                (byte)((target >> 16) & 0xFF), 
                (byte)((target >> 8) & 0xFF), 
                (byte)(target & 0xFF)
            },
            APIClass.CTRL.get(1)
        );
    }

    private void setConfig(int index, int value, Error err) {
        if (!setMotorConfig(index, value)) 
            addError(err);
    }

    private boolean setMotorConfig(int index, int value) {     

        if (CANAPIJNI.writeCANPacketNoThrow(
            deviceHandle, 
            new byte[] {
                (byte)((value >> 24) & 0xFF), 
                (byte)((value >> 16) & 0xFF), 
                (byte)((value >> 8) & 0xFF), 
                (byte)(value & 0xFF),
                (byte)((index >> 24) & 0xFF), 
                (byte)((index >> 16) & 0xFF), 
                (byte)((index >> 8) & 0xFF), 
                (byte)(index & 0xFF)
            },
            APIClass.CTRL.get(0)
        ) != 0) return false;

        CANData dataBuffer = new CANData();
        long rioTime = RobotController.getFPGATime();

        // Checks if robot disabled because 1. blocking isn't as much of an issue and 2. rio start up times
        while ((RobotController.getFPGATime() - rioTime) < (RobotState.isDisabled() ? 50_000 : 5_000))
        {
            CANAPIJNI.readCANPacketNew(
                deviceHandle, 
                APIClass.STATUS.get(5),
                dataBuffer
            );

            if (dataBuffer.data[0] == 0x69) return true;
            //6E696365
        }
        
        // System.out.println("\nThriftyWarning: Failed to set configuration " + index + ".\n");
        return false;
    }

    private int translateOutput(double input) {
        return (int)(10_000 * input);
    }
 
    private double limitRange(double min, double max, double input) {
        return Math.min(Math.max(input, min), max);
    }
}

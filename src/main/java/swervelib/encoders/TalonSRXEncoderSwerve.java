package swervelib.encoders;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import swervelib.motors.SwerveMotor;

/**
 * Talon SRX integrated encoder
 */
public class TalonSRXEncoderSwerve extends SwerveAbsoluteEncoder {

    private final double sensorUnitsPerDegree = 4096.0 / 360.0;
    private final WPI_TalonSRX talon;

    public TalonSRXEncoderSwerve(SwerveMotor motor, FeedbackDevice feedbackDevice) {
        if (motor.getMotor() instanceof WPI_TalonSRX talon) {
            this.talon = talon;
            talon.configSelectedFeedbackSensor(feedbackDevice);
        } 
        else {
            throw new RuntimeException("Motor given to instantiate TalonSRXEncoder is not a WPI_TalonSRX");
        }
    }

    @Override
    public void factoryDefault() {
        // Handled in TalonSRXSwerve
    }

    @Override
    public void clearStickyFaults() {
        // Handled in TalonSRXSwerve
    }

    @Override
    public void configure(boolean inverted) {
        talon.setSensorPhase(inverted);
    }

    @Override
    public double getAbsolutePosition() {
        return talon.getSelectedSensorPosition() * sensorUnitsPerDegree;
    }

    @Override
    public Object getAbsoluteEncoder() {
        return talon;
    }

    @Override
    public boolean setAbsoluteEncoderOffset(double offset) {
        talon.setSelectedSensorPosition(talon.getSelectedSensorPosition() + offset / sensorUnitsPerDegree);
        return true;
    }

    @Override
    public double getVelocity() {
        return talon.getSelectedSensorVelocity() * 10 * sensorUnitsPerDegree;
    }
    
}

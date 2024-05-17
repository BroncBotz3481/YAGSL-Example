package swervelib.encoders;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import swervelib.motors.SwerveMotor;

/**
 * Talon SRX integrated encoder
 */
public class TalonSRXEncoderSwerve extends SwerveAbsoluteEncoder {

    private final double degreesPerSensorUnit;
    private final WPI_TalonSRX talon;

    public TalonSRXEncoderSwerve(SwerveMotor motor, FeedbackDevice feedbackDevice) {
        if (motor.getMotor() instanceof WPI_TalonSRX talon) {
            this.talon = talon;
            talon.configSelectedFeedbackSensor(feedbackDevice);
            // https://v5.docs.ctr-electronics.com/en/stable/ch14_MCSensor.html#sensor-resolution
            degreesPerSensorUnit = switch (feedbackDevice) {
                case Analog -> 360.0 / 1024.0;
                default -> 360.0 / 4096.0;
            };
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
        return talon.getSelectedSensorPosition() * degreesPerSensorUnit;
    }

    @Override
    public Object getAbsoluteEncoder() {
        return talon;
    }

    @Override
    public boolean setAbsoluteEncoderOffset(double offset) {
        talon.setSelectedSensorPosition(talon.getSelectedSensorPosition() + offset / degreesPerSensorUnit);
        return true;
    }

    @Override
    public double getVelocity() {
        return talon.getSelectedSensorVelocity() * 10 * degreesPerSensorUnit;
    }
    
}

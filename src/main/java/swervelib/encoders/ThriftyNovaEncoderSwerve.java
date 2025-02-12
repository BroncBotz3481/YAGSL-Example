package swervelib.encoders;

import com.thethriftybot.Conversion;
import com.thethriftybot.Conversion.PositionUnit;
import com.thethriftybot.Conversion.VelocityUnit;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.EncoderType;

import swervelib.motors.SwerveMotor;
import swervelib.motors.ThriftyNovaSwerve;

/**
 * Thrifty Nova absolute encoder, attached through the data port.
 */
public class ThriftyNovaEncoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * The absolute encoder is directly interfaced through the Thrifty Nova motor.
   */
  protected ThriftyNova motor;
  /**
   * Inversion state of the attached encoder.
   */
  protected boolean           inverted = false;
  /**
   * Offset of the absolute encoder.
   */
  protected double            offset   = 0.0;
  /**
   * Position conversion object for the motor encoder
   */
  private       Conversion    positionConversion;
  /**
   * Velocity conversion object for the motor encoder
   */
  private       Conversion    velocityConversion;
    
  /**
   * Create the {@link ThriftyNovaEncoderSwerve} object as an absolute encoder from the {@link ThriftyNovaSwerve}
   * motor.
   *
   * @param motor {@link SwerveMotor} through which to interface with the attached encoder .
   */
  public ThriftyNovaEncoderSwerve(SwerveMotor motor)
  {
    this.motor = (ThriftyNova) motor.getMotor();
    positionConversion = new Conversion(PositionUnit.DEGREES, EncoderType.ABS);
    velocityConversion = new Conversion(VelocityUnit.DEGREES_PER_SEC, EncoderType.ABS);
    this.motor.useEncoderType(EncoderType.ABS);
  }

  @Override
  public void close()
  {
    // ThriftyNova encoder gets closed with the motor
    // I don't think an encoder getting closed should 
    // close the entire motor so i will keep this empty
  }

  /**
   * Set factory default.
   */
  @Override
  public void factoryDefault()
  {
  }

  /**
   * Clear sticky faults.
   */
  @Override
  public void clearStickyFaults()
  {
  }

  /**
   * Configure the absolute encoder.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {
    this.inverted = inverted;
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    double rawMotor = motor.getPosition();
    double convertedMotor = positionConversion.fromMotor(rawMotor);
    return (convertedMotor + offset) * (inverted ? -1.0 : 1.0);
  }

  /**
   * Get the instantiated absolute encoder Object.
   */
  @Override
  public Object getAbsoluteEncoder()
  {
    return null;
  }

  /**
   * Set the absolute encoder offset.
   *
   * @param offset offset in degrees from [0, 360).
   * @return true if successful.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    this.offset = offset;
    return true;
  }

  /**
   * Get the absolute encoder velocity. WARNING: Angular velocity is generally not measurable at high speeds.
   *
   * @return Velocity in degrees per second.
   */
  @Override
  public double getVelocity()
  {
    return velocityConversion.fromMotor(motor.getVelocity()) * (inverted ? -1.0 : 1.0);
  }
}

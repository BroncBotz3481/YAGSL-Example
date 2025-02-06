package swervelib.encoders;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import swervelib.motors.SparkMaxBrushedMotorSwerve;
import swervelib.motors.SparkMaxSwerve;
import swervelib.motors.SparkSwerve;
import swervelib.motors.SwerveMotor;

/**
 * SparkBase absolute encoder, attached through the data port analog pin.
 */
public class SparkAnalogEncoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * The {@link SparkAnalogSensor} representing the duty cycle encoder attached to the SparkBase analog port.
   */
  public SparkAnalogSensor encoder;

  /**
   * {@link swervelib.motors.SparkMaxSwerve} or {@link swervelib.motors.SparkMaxBrushedMotorSwerve} object.
   */
  protected final SwerveMotor motor;

  /**
   * An {@link Alert} for if there is a failure configuring the encoder.
   */
  protected final Alert failureConfiguring;

  /**
   * An {@link Alert} for if the absolute encoder does not support integrated offsets.
   */
  protected final Alert doesNotSupportIntegratedOffsets;

  /**
   * Create the {@link SparkAnalogEncoderSwerve} object as a analog sensor from the {@link SparkBase}
   * motor data port analog pin.
   *
   * @param motor Motor to create the encoder from.
   * @param maxVoltage Maximum voltage for analog input reading.
   */
  public SparkAnalogEncoderSwerve(SwerveMotor motor, double maxVoltage)
  {
    if (motor.getMotor() instanceof SparkBase)
    {
      this.motor = motor;
      encoder = ((SparkBase) motor.getMotor()).getAnalog();
      motor.setAbsoluteEncoder(this);
      setConversionFactor(360.0 / maxVoltage);
    } else
    {
      throw new RuntimeException("Motor given to instantiate SparkAnalogEncoderSwerve is not a SparkBase");
    }
    failureConfiguring = new Alert(
        "Encoders", 
        "Failure configuring SparkBase Analog Encoder", 
        AlertType.kWarning);
    doesNotSupportIntegratedOffsets = new Alert(
        "Encoders",
        "SparkBase Analog Sensors do not support integrated offsets",
        AlertType.kWarning);
  }

  @Override
  public void close()
  {
    // SPARK MAX Analog encoder gets closed with the motor
    // I don't think an encoder getting closed should 
    // close the entire motor so i will keep this empty
    // sparkMax.close();
  }

  /**
   * Set the conversion factor of the {@link SparkMaxAnalogEncoderSwerve}.
   *
   * @param conversionFactor Position conversion factor from ticks to unit.
   */
  public void setConversionFactor(double conversionFactor)
  {
    SparkBaseConfig cfg = null;
    if (motor instanceof SparkMaxSwerve)
    {
      cfg = ((SparkMaxSwerve) motor).getConfig();
    } else if (motor instanceof SparkMaxBrushedMotorSwerve)
    {
      cfg = ((SparkMaxBrushedMotorSwerve) motor).getConfig();
    }
    if (cfg != null)
    {
      cfg.closedLoop.feedbackSensor(FeedbackSensor.kAnalogSensor);

      cfg.signals
          .analogVelocityAlwaysOn(true)
          .analogVoltageAlwaysOn(true)
          .analogPositionAlwaysOn(true)
          .analogVoltagePeriodMs(20)
          .analogPositionPeriodMs(20)
          .analogVelocityPeriodMs(20);

      cfg.analogSensor
          .positionConversionFactor(conversionFactor)
          .velocityConversionFactor(conversionFactor / 60);
    }
    if (motor instanceof SparkMaxSwerve)
    {
      ((SparkMaxSwerve) motor).updateConfig(cfg);
    } else if (motor instanceof SparkMaxBrushedMotorSwerve)
    {
      ((SparkMaxBrushedMotorSwerve) motor).updateConfig((SparkMaxConfig) cfg);
    }
  }

  /**
   * Reset the encoder to factory defaults.
   */
  @Override
  public void factoryDefault()
  {
    // Do nothing
  }

  /**
   * Clear sticky faults on the encoder. */
  @Override
  public void clearStickyFaults()
  {
    // Do nothing
  }

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {
    if (motor instanceof SparkSwerve) {
      var sparkBase = (SparkSwerve) motor;
      SparkBaseConfig cfg = sparkBase.getConfig();
      cfg.analogSensor.inverted(inverted);
      sparkBase.updateConfig(cfg);
    } else if (motor instanceof SparkMaxBrushedMotorSwerve) {
      var sparkMax = (SparkMaxBrushedMotorSwerve) motor;
      SparkMaxConfig cfg = sparkMax.getConfig();
      cfg.analogSensor.inverted(inverted);
      sparkMax.updateConfig(cfg);
    }
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    return encoder.getPosition();
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  @Override
  public Object getAbsoluteEncoder()
  {
    return encoder;
  }

  /**
   * Sets the Absolute Encoder offset at the Encoder Level.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    doesNotSupportIntegratedOffsets.set(true);
    return false;
  }

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  @Override
  public double getVelocity()
  {
    return encoder.getVelocity();
  }
}

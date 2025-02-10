package swervelib.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import swervelib.motors.SparkFlexSwerve;
import swervelib.motors.SparkMaxBrushedMotorSwerve;
import swervelib.motors.SparkMaxSwerve;
import swervelib.motors.SparkSwerve;
import swervelib.motors.SwerveMotor;

/**
 * SparkBase absolute encoder, attached through the data port.
 */
public class SparkEncoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * The {@link AbsoluteEncoder} representing the duty cycle encoder attached to the SparkBase.
   */
  public final SparkAbsoluteEncoder encoder;

  /**
   * {@link SparkMaxBrushedMotorSwerve} or {@link SparkSwerve} instance.
   */
  protected final SwerveMotor motor;

  /**
   * An {@link Alert} for if there is a failure configuring the encoder.
   */
  protected final Alert failureConfiguring;

  /**
   * An {@link Alert} for if there is a failure configuring the encoder offset.
   */
  protected final Alert offsetFailure;

  /**
   * Create the {@link SparkEncoderSwerve} object as a duty cycle from the {@link
   * com.revrobotics.spark.SparkBase} motor.
   *
   * @param motor Motor to create the encoder from.
   * @param conversionFactor The conversion factor to set if the output is not from 0 to 360.
   */
  public SparkEncoderSwerve(SwerveMotor motor, int conversionFactor)
  {
    failureConfiguring = new Alert(
      "Encoders", 
      "Failure configuring SparkBase Absolute Encoder", 
      AlertType.kWarning);
    offsetFailure = new Alert(
      "Encoders", 
      "Failure to set Absolute Encoder Offset", 
      AlertType.kWarning);
    if (motor.getMotor() instanceof SparkBase)
    {
      this.motor = motor;
      encoder = ((SparkBase) motor.getMotor()).getAbsoluteEncoder();
      motor.setAbsoluteEncoder(this);
      setConversionFactor(conversionFactor);
    } else
    {
      throw new RuntimeException("Motor given to instantiate SparkEncoderSwerve is not a SparkBase");
    }
  }

  @Override
  public void close()
  {
      // SPARK MAX/FLEX encoder gets closed with the motor
      // I don't think an encoder getting closed should 
      // close the entire motor so i will keep this empty
      // sparkFlex.close();
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
   * Clear sticky faults on the encoder.
   */
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
    if (motor instanceof SparkSwerve)
    {
      var sparkBase = (SparkSwerve) motor;
      SparkBaseConfig cfg = sparkBase.getConfig();
      cfg.absoluteEncoder.inverted(inverted);
      sparkBase.updateConfig(cfg);
    }
  }

  /**
   * Set the conversion factor of the {@link SparkEncoderSwerve}.
   *
   * @param conversionFactor Position conversion factor from ticks to unit.
   */
  public void setConversionFactor(double conversionFactor)
  {
    // By default the SparkMax relays the info from the duty cycle encoder to the roborio every 200ms on CAN frame 5
    // This needs to be set to 20ms or under to properly update the swerve module position for odometry
    // Configuration taken from 3005, the team who helped develop the Max Swerve:
    // https://github.com/FRC3005/Charged-Up-2023-Public/blob/2b6a7c695e23edebafa27a76cf639a00f6e8a3a6/src/main/java/frc/robot/subsystems/drive/REVSwerveModule.java#L227-L244
    // Some of the frames can probably be adjusted to decrease CAN utilization, with 65535 being the max.
    // From testing, 20ms on frame 5 sometimes returns the same value while constantly powering the azimuth but 8ms may be overkill,
    // with limited testing 19ms did not return the same value while the module was constatntly rotating.

    SparkBaseConfig cfg = null;
    if (motor instanceof SparkSwerve)
    {
      cfg = ((SparkSwerve) motor).getConfig();
    } else if (motor instanceof SparkMaxBrushedMotorSwerve)
    {
      cfg = ((SparkMaxBrushedMotorSwerve) motor).getConfig();
    }
    if (cfg != null)
    {
      cfg.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

      cfg.signals
          .absoluteEncoderPositionAlwaysOn(true)
          .absoluteEncoderPositionPeriodMs(20);

      cfg.absoluteEncoder
          .positionConversionFactor(conversionFactor)
          .velocityConversionFactor(conversionFactor / 60);
    }
    if (motor instanceof SparkSwerve)
    {
      ((SparkMaxSwerve) motor).updateConfig(cfg);
    } else if (motor instanceof SparkMaxBrushedMotorSwerve)
    {
      ((SparkMaxBrushedMotorSwerve) motor).updateConfig((SparkMaxConfig) cfg);
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
   * Sets the Absolute Encoder Offset inside of the SparkBase's Memory.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    if (motor instanceof SparkFlexSwerve)
    {
      var sparkFlex = (SparkFlexSwerve) motor;
      SparkBaseConfig cfg = sparkFlex.getConfig();
      cfg.absoluteEncoder.zeroOffset(offset);
      sparkFlex.updateConfig(cfg);
      return true;
    }
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

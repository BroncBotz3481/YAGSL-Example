package frc.robot.subsystems.swervedrive2.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants.Drivebase.EncoderConversions;
import frc.robot.subsystems.swervedrive2.parser.PIDFConfig;

public class SparkMaxSwerve extends SwerveMotor
{

  /**
   * SparkMAX Instance.
   */
  public CANSparkMax           motor;
  /**
   * Integrated encoder.
   */
  public RelativeEncoder       encoder;
  /**
   * Closed-loop PID controller.
   */
  public SparkMaxPIDController pid;

  /**
   * Initialize the swerve motor.
   *
   * @param id           CAN ID of the SparkMax
   * @param isDriveMotor Is the motor being initialized a drive motor?
   */
  public SparkMaxSwerve(int id, boolean isDriveMotor)
  {
    this.isDriveMotor = isDriveMotor;
    motor = new CANSparkMax(id, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pid = motor.getPIDController();
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    motor.restoreFactoryDefaults();
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    motor.clearFaults();
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   */
  @Override
  public void configureIntegratedEncoder()
  {
    if (isDriveMotor)
    {
      encoder.setPositionConversionFactor(EncoderConversions.METERS_PER_MOTOR_ROTATION);
      encoder.setVelocityConversionFactor(EncoderConversions.METERS_PER_MOTOR_ROTATION / 60);
    } else
    {
      encoder.setPositionConversionFactor(EncoderConversions.DEGREES_PER_STEERING_ROTATION);
      encoder.setVelocityConversionFactor(EncoderConversions.DEGREES_PER_STEERING_ROTATION / 60);
    }
  }

  /**
   * Configure the PIDF values for the closed loop controller.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    int pidSlot = isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    pid.setP(config.kP, pidSlot);
    pid.setI(config.kI, pidSlot);
    pid.setD(config.kD, pidSlot);
    pid.setFF(config.kF, pidSlot);
    pid.setIZone(config.IZ, pidSlot);
  }

  /**
   * Configure the PID wrapping for the position closed loop controller.
   *
   * @param minInput Minimum PID input.
   * @param maxInput Maximum PID input.
   */
  @Override
  public void configurePIDWrapping(double minInput, double maxInput)
  {
    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMinInput(minInput);
    pid.setPositionPIDWrappingMaxInput(maxInput);
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    motor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    motor.setInverted(inverted);
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash()
  {
    motor.burnFlash();
  }

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput)
  {
    motor.set(percentOutput);
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in MPS or Angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   */
  @Override
  public void setReference(double setpoint, double feedforward)
  {
    int pidSlot = isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    pid.setReference(setpoint, isDriveMotor ? ControlType.kVelocity : ControlType.kPosition, pidSlot, feedforward);
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity
   */
  @Override
  public double getVelocity()
  {
    return encoder.getVelocity();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position
   */
  @Override
  public double getPosition()
  {
    return encoder.getPosition();
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position.
   */
  @Override
  public void setPosition(double position)
  {
    encoder.setPosition(position);
  }

  /**
   * REV Slots for PID configuration.
   */
  enum SparkMAX_slotIdx
  {
    Position, Velocity, Simulation
  }
}
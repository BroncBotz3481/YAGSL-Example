package swervelib.motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.motors.SparkMaxSwerve.SparkMAX_slotIdx;
import swervelib.parser.PIDFConfig;

/**
 * Brushed motor control with SparkMax.
 */
public class SparkMaxBrushedMotorSwerve extends SwerveMotor
{

  /**
   * SparkMAX Instance.
   */
  public CANSparkMax motor;

  /**
   * Absolute encoder attached to the SparkMax (if exists)
   */
  public  AbsoluteEncoder       absoluteEncoder;
  /**
   * Integrated encoder.
   */
  public  RelativeEncoder       encoder;
  /**
   * Closed-loop PID controller.
   */
  public  SparkMaxPIDController pid;
  /**
   * Factory default already occurred.
   */
  private boolean               factoryDefaultOccurred = false;

  /**
   * Initialize the swerve motor.
   *
   * @param motor                  The SwerveMotor as a SparkMax object.
   * @param isDriveMotor           Is the motor being initialized a drive motor?
   * @param encoderType            {@link Type} of encoder to use for the {@link CANSparkMax} device.
   * @param countsPerRevolution    The number of encoder pulses for the {@link Type} encoder per revolution.
   * @param useDataPortQuadEncoder Use the encoder attached to the data port of the spark max for a quadrature encoder.
   */
  public SparkMaxBrushedMotorSwerve(CANSparkMax motor, boolean isDriveMotor, Type encoderType, int countsPerRevolution,
                                    boolean useDataPortQuadEncoder)
  {
    // Drive motors **MUST** have an encoder attached.
    if (isDriveMotor && encoderType == Type.kNoSensor)
    {
      DriverStation.reportError("Cannot use motor without encoder.", true);
      throw new RuntimeException("Cannot use SparkMAX as a drive motor without an encoder attached.");
    }

    // Hall encoders can be used as quadrature encoders.
    if (encoderType == Type.kHallSensor)
    {
      encoderType = Type.kQuadrature;
    }

    this.motor = motor;
    this.isDriveMotor = isDriveMotor;

    factoryDefaults();
    clearStickyFaults();

    // Get the onboard PID controller.
    pid = motor.getPIDController();

    // If there is a sensor attached to the data port or encoder port set the relative encoder.
    if (isDriveMotor || (encoderType != Type.kNoSensor || useDataPortQuadEncoder))
    {
      this.encoder = useDataPortQuadEncoder ?
                     motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, countsPerRevolution) :
                     motor.getEncoder(encoderType, countsPerRevolution);

      // Configure feedback of the PID controller as the integrated encoder.
      pid.setFeedbackDevice(encoder);
    }

    motor.setCANTimeout(0); // Spin off configurations in a different thread.
  }

  /**
   * Initialize the {@link SwerveMotor} as a {@link CANSparkMax} connected to a Brushless Motor.
   *
   * @param id                     CAN ID of the SparkMax.
   * @param isDriveMotor           Is the motor being initialized a drive motor?
   * @param encoderType            {@link Type} of encoder to use for the {@link CANSparkMax} device.
   * @param countsPerRevolution    The number of encoder pulses for the {@link Type} encoder per revolution.
   * @param useDataPortQuadEncoder Use the encoder attached to the data port of the spark max for a quadrature encoder.
   */
  public SparkMaxBrushedMotorSwerve(int id, boolean isDriveMotor, Type encoderType, int countsPerRevolution,
                                    boolean useDataPortQuadEncoder)
  {
    this(new CANSparkMax(id, MotorType.kBrushed), isDriveMotor, encoderType, countsPerRevolution,
         useDataPortQuadEncoder);
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    motor.enableVoltageCompensation(nominalVoltage);
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public void setCurrentLimit(int currentLimit)
  {
    motor.setSmartCurrentLimit(currentLimit);
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    motor.setOpenLoopRampRate(rampRate);
    motor.setClosedLoopRampRate(rampRate);
  }

  /**
   * Get the motor object from the module.
   *
   * @return Motor object.
   */
  @Override
  public Object getMotor()
  {
    return motor;
  }

  /**
   * Queries whether the absolute encoder is directly attached to the motor controller.
   *
   * @return connected absolute encoder state.
   */
  @Override
  public boolean isAttachedAbsoluteEncoder()
  {
    return absoluteEncoder != null;
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    if (!factoryDefaultOccurred)
    {
      motor.restoreFactoryDefaults();
      factoryDefaultOccurred = true;
    }
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
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   * @return The {@link SwerveMotor} for easy instantiation.
   */
  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder)
  {
    if (encoder.getAbsoluteEncoder() instanceof AbsoluteEncoder)
    {
      absoluteEncoder = (AbsoluteEncoder) encoder.getAbsoluteEncoder();
      pid.setFeedbackDevice(absoluteEncoder);
    }
    if (absoluteEncoder == null && this.encoder == null)
    {
      DriverStation.reportError("An encoder MUST be defined to work with a SparkMAX", true);
      throw new RuntimeException("An encoder MUST be defined to work with a SparkMAX");
    }
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply.
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    if (absoluteEncoder == null)
    {
      encoder.setPositionConversionFactor(positionConversionFactor);
      encoder.setVelocityConversionFactor(positionConversionFactor / 60);

      // Taken from
      // https://github.com/frc3512/SwerveBot-2022/blob/9d31afd05df6c630d5acb4ec2cf5d734c9093bf8/src/main/java/frc/lib/util/CANSparkMaxUtil.java#L67
      configureCANStatusFrames(10, 20, 20, 500, 500);
    } else
    {
      absoluteEncoder.setPositionConversionFactor(positionConversionFactor);
      absoluteEncoder.setVelocityConversionFactor(positionConversionFactor / 60);
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
    int pidSlot =
        isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    pid.setP(config.p, pidSlot);
    pid.setI(config.i, pidSlot);
    pid.setD(config.d, pidSlot);
    pid.setFF(config.f, pidSlot);
    pid.setIZone(config.iz, pidSlot);
    pid.setOutputRange(config.output.min, config.output.max, pidSlot);
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
   * Set the CAN status frames.
   *
   * @param CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
   * @param CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
   * @param CANStatus2 Motor Position
   * @param CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
   * @param CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
   */
  public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4)
  {
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4);
    // TODO: Configure Status Frame 5 and 6 if necessary
    //  https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
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
    int pidSlot =
        isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    pid.setReference(
        setpoint,
        isDriveMotor ? ControlType.kVelocity : ControlType.kPosition,
        pidSlot,
        feedforward);
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in meters per second or angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   * @param position    Only used on the angle motor, the position of the motor in degrees.
   */
  @Override
  public void setReference(double setpoint, double feedforward, double position)
  {
    setReference(setpoint, feedforward);
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity
   */
  @Override
  public double getVelocity()
  {
    return absoluteEncoder == null ? encoder.getVelocity() : absoluteEncoder.getVelocity();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position
   */
  @Override
  public double getPosition()
  {
    return absoluteEncoder == null ? encoder.getPosition() : absoluteEncoder.getPosition();
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position.
   */
  @Override
  public void setPosition(double position)
  {
    if (absoluteEncoder == null)
    {
      encoder.setPosition(position);
    }
  }
}

package swervelib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;
import swervelib.simulation.ctre.PhysicsSim;

/**
 * {@link com.ctre.phoenix.motorcontrol.can.TalonFX} Swerve Motor.
 */
public class TalonFXSwerve extends SwerveMotor
{

  /**
   * Factory default already occurred.
   */
  private final boolean factoryDefaultOccurred = false;
  /**
   * TalonFX motor controller.
   */
  WPI_TalonFX motor;
  /**
   * Current TalonFX configuration.
   */
  private final TalonFXConfiguration configuration            = new TalonFXConfiguration();
  /**
   * Whether the absolute encoder is integrated.
   */
  private final boolean              absoluteEncoder          = false;
  /**
   * The position conversion factor to convert raw sensor units to Meters Per 100ms, or Ticks to Degrees.
   */
  private       double               positionConversionFactor = 1;
  /**
   * If the TalonFX configuration has changed.
   */
  private       boolean              configChanged            = true;

  /**
   * Constructor for TalonFX swerve motor.
   *
   * @param motor        Motor to use.
   * @param isDriveMotor Whether this motor is a drive motor.
   */
  public TalonFXSwerve(WPI_TalonFX motor, boolean isDriveMotor)
  {
    this.isDriveMotor = isDriveMotor;
    this.motor = motor;

    factoryDefaults();
    clearStickyFaults();

    if (!Robot.isReal())
    {
      PhysicsSim.getInstance().addTalonFX(motor, .00001, 6800 * 10);
    }
  }

  /**
   * Construct the TalonFX swerve motor given the ID and CANBus.
   *
   * @param id           ID of the TalonFX on the CANBus.
   * @param canbus       CANBus on which the TalonFX is on.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public TalonFXSwerve(int id, String canbus, boolean isDriveMotor)
  {
    this(new WPI_TalonFX(id, canbus), isDriveMotor);
  }

  /**
   * Construct the TalonFX swerve motor given the ID.
   *
   * @param id           ID of the TalonFX on the canbus.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public TalonFXSwerve(int id, boolean isDriveMotor)
  {
    this(new WPI_TalonFX(id), isDriveMotor);
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    if (!factoryDefaultOccurred)
    {
      motor.configFactoryDefault();
      motor.setSensorPhase(true);
    }
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    motor.clearStickyFaults();
  }


  /**
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   */
  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder)
  {
    /*if (encoder.getAbsoluteEncoder() instanceof CANCoder)
    {
      configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
      configuration.remoteFilter0.remoteSensorDeviceID = ((CANCoder) encoder.getAbsoluteEncoder()).getDeviceID();
      configuration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
//      motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
//      motor.configRemoteFeedbackFilter((CANCoder) encoder.getAbsoluteEncoder(),
//                                       CTRE_remoteSensor.REMOTE_SENSOR_0.ordinal());
      configChanged = true;
      absoluteEncoder = true;
    }*/
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply for position. <p><br /> Degrees: <br />
   *                                 <code>
   *                                 360 / (angleGearRatio * encoderTicksPerRotation)
   *                                 </code><br /></p>
   *                                 <p><br />Meters:<br />
   *                                 <code>
   *                                 (Math.PI * wheelDiameter) / (driveGearRatio * encoderTicksPerRotation)
   *                                 </code>
   *                                 </p>
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    this.positionConversionFactor = positionConversionFactor;
//    configuration.primaryPID.selectedFeedbackCoefficient = positionConversionFactor;
//    configChanged = true;
//    motor.configSelectedFeedbackCoefficient(positionConversionFactor);
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
//    int slotIdx = isDriveMotor ? CTRE_slotIdx.Velocity.ordinal() : CTRE_slotIdx.Turning.ordinal();
//    slotIdx = 0;
//    motor.config_kP(slotIdx, config.p);
//    motor.config_kI(slotIdx, config.i);
//    motor.config_kD(slotIdx, config.d);
//    motor.config_kF(slotIdx, config.f);
//    motor.config_IntegralZone(slotIdx, config.iz);
//    motor.configClosedLoopPeakOutput(slotIdx, config.output.max);
    configuration.slot0.kP = config.p;
    configuration.slot0.kI = config.i;
    configuration.slot0.kD = config.d;
    configuration.slot0.kF = config.f;
    configuration.slot0.integralZone = config.iz;
    configuration.slot0.closedLoopPeakOutput = config.output.max;
    configChanged = true;
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
    // Do nothing
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    motor.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
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
    if (configChanged)
    {
      motor.configAllSettings(configuration);
      configChanged = false;
    }
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
   * Convert the setpoint into native sensor units.
   *
   * @param setpoint Setpoint to mutate. In meters per second or degrees.
   * @return Setpoint as native sensor units. Encoder ticks per 100ms, or Encoder tick.
   */
  public double convertToNativeSensorUnits(double setpoint)
  {
    setpoint = isDriveMotor ? (setpoint * .1) : setpoint;
    return setpoint / positionConversionFactor;
  }

  /**
   * Normalize the given value depending on the module. If angle motor then value will be normalized.
   *
   * @param value Meters or Degrees
   * @return Normalized value.
   */
  public double normalize(double value)
  {
    return isDriveMotor ? value : (SwerveMath.normalizeAngle(value));
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
    if (!Robot.isReal())
    {
      PhysicsSim.getInstance().run();
    }

    burnFlash();

    if (!isDriveMotor)
    {
      System.out.println("The angle motor is " + motor.getDeviceID());
      System.out.println("Setpoint " + normalize(setpoint));
      System.out.println("Current point " + (getPosition()));

      // Credit to Team 3538!
      // Inspired from https://github.com/FRC-Team3538/FRC2023-beta/blob/java/src/main/java/io/robojackets/subsystems/SwerveModule.java#L176-L199
      Rotation2d normalized_target_angle = Rotation2d.fromDegrees(SwerveMath.normalizeAngle(setpoint));
      Rotation2d actual_current_angle    = Rotation2d.fromDegrees(getRawPosition());
      Rotation2d normalized_actual_current_angle = Rotation2d.fromDegrees(
          SwerveMath.normalizeAngle(actual_current_angle.getDegrees()));

      Rotation2d target_diff = normalized_target_angle.minus(normalized_actual_current_angle);

      setpoint = Math.toDegrees(actual_current_angle.getRadians() + target_diff.getRadians());
    }

    motor.set(isDriveMotor ? ControlMode.Velocity : ControlMode.Position,
              convertToNativeSensorUnits(setpoint),
              DemandType.ArbitraryFeedForward,
              feedforward);
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity in Meters Per Second, or Degrees per Second.
   */
  @Override
  public double getVelocity()
  {
    return normalize((motor.getSelectedSensorVelocity() * 10) * positionConversionFactor);
  }

  /**
   * Get the raw position.
   *
   * @return Position in meters or degrees.
   */
  public double getRawPosition()
  {
    return motor.getSelectedSensorPosition() * positionConversionFactor;
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position in Meters or Degrees.
   */
  @Override
  public double getPosition()
  {
    return normalize(getRawPosition());
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position. Should be angle in degrees or meters.
   */
  @Override
  public void setPosition(double position)
  {
    if (!absoluteEncoder && Robot.isReal())
    {
      motor.setSelectedSensorPosition(convertToNativeSensorUnits(position));
    }
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
//    motor.enableVoltageCompensation(true);
//    motor.configVoltageCompSaturation(nominalVoltage);
    configuration.voltageCompSaturation = nominalVoltage;
    configChanged = true;
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
//    SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
//    motor.configGetSupplyCurrentLimit(config);
//    config.currentLimit = currentLimit;
//    config.enable = true;
//    motor.configSupplyCurrentLimit(config);
    configuration.supplyCurrLimit.currentLimit = currentLimit;
    configuration.supplyCurrLimit.enable = true;
    configChanged = true;
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
//    motor.configClosedloopRamp(rampRate);
//    motor.configOpenloopRamp(rampRate);
    configuration.closedloopRamp = rampRate;
    configuration.openloopRamp = rampRate;
    configChanged = true;
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
    return absoluteEncoder;
  }

}

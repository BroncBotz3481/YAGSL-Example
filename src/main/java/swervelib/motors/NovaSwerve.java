package swervelib.motors;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.PIDSlot;

import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry;

public class NovaSwerve extends SwerveMotor
{

  /**
   * Wait time for status frames to show up.
   */
  public static double             STATUS_TIMEOUT_SECONDS  = 0.02;
  /**
   * Factory default already occurred.
   */
  private final boolean            factoryDefaultOccurred  = false;
  /**
   * Whether the absolute encoder is integrated.
   */
  private final boolean            absoluteEncoder         = false;
  /**
   * Nova motor controller.
   */
  private final ThriftyNova motor;
  /**
   * Conversion factor for the motor.
   */
  private double               conversionFactor;
  /**

  /**
   * Constructor for Nova swerve motor controller.
   *
   * @param motor        Motor to use.
   * @param isDriveMotor Whether this motor is a drive motor.
   */
  public NovaSwerve(ThriftyNova motor, boolean isDriveMotor)
  {
    this.isDriveMotor = isDriveMotor;
    this.motor = motor;

    factoryDefaults();
    clearStickyFaults();
  }

  /**
   * Construct the Nova swerve motor controller given the ID.
   *
   * @param id           ID of the TalonFX on the CANBus.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public NovaSwerve(int id, boolean isDriveMotor)
  {
    this(new ThriftyNova(id), isDriveMotor);
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    if (!factoryDefaultOccurred)
    {
      //From: https://docs.thethriftybot.com/thrifty-nova/gqCPUYXcVoOZ4KW3DqIr/software-resources/configure-controller-settings/factory-default
      motor.setInverted(false);
      motor.setBrakeMode(false);
      motor.canFreq.setControl(0.02).setCurrent(0.20).setFault(0.25).setQuadSensor(0.01).setSensor(0.01);
      motor.setMaxOutput(1, 1);
      motor.setRampUp(0.1);
      motor.setRampDown(0.1);
      motor.setMaxCurrent(CurrentType.STATOR, 40); //Not sure what current type
      motor.enableHardLimits(false);
      motor.enableSoftLimits(false);
      motor.pid0.setP(0);
      motor.pid0.setI(0);
      motor.pid0.setD(0);
      motor.pid0.setFF(0);
      motor.pid1.setP(0);
      motor.pid1.setI(0);
      motor.pid1.setD(0);
      motor.pid1.setFF(0);
      motor.setSoftLimits(0, 0);
    }
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    motor.clearErrors();
  }

  /**
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   */
  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder)
  {
    // Does not support.
    //motor.useEncoderType(EncoderType.ABS);
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply for position.
   *                                 <p><br>
   *                                 Degrees: <br>
   *                                 <code>
   *                                 360 / (angleGearRatio * encoderTicksPerRotation)
   *                                 </code><br>
   *                                 <p><br>
   *                                 Meters:<br>
   *                                 <code>
   *                                 (Math.PI * wheelDiameter) / (driveGearRatio * encoderTicksPerRotation)
   *                                 </code>
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    //Not Supporting setting conversion factors
    motor.useEncoderType(EncoderType.INTERNAL);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1 Applied Motor Output, Fault Information, Limit Switch Information
   */
  public void configureCANStatusFrames(int CANStatus1)
  {
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1       Applied Motor Output, Fault Information, Limit Switch Information
   * @param CANStatus2       Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), Brushed Supply Current
   *                         Measurement, Sticky Fault Information
   * @param CANStatus3       Quadrature Information
   * @param CANStatus4       Analog Input, Supply Battery Voltage, Controller Temperature
   * @param CANStatus8       Pulse Width Information
   * @param CANStatus10      Motion Profiling/Motion Magic Information
   * @param CANStatus12      Selected Sensor Position (Aux PID 1), Selected Sensor Velocity (Aux PID 1)
   * @param CANStatus13      PID0 (Primary PID) Information
   * @param CANStatus14      PID1 (Auxiliary PID) Information
   * @param CANStatus21      Integrated Sensor Position (Talon FX), Integrated Sensor Velocity (Talon FX)
   * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless Stator Current Measurement
   */
  public void configureCANStatusFrames(
      int CANStatus1,
      int CANStatus2,
      int CANStatus3,
      int CANStatus4,
      int CANStatus8,
      int CANStatus10,
      int CANStatus12,
      int CANStatus13,
      int CANStatus14,
      int CANStatus21,
      int CANStatusCurrent)
  {
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, CANStatus2);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, CANStatus3);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, CANStatus4);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, CANStatus8);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, CANStatus10);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, CANStatus12);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, CANStatus13);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, CANStatus14);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, CANStatus21);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current,
    // CANStatusCurrent);

    // TODO: Configure Status Frame 2 thru 21 if necessary
    // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html#setting-status-frame-periods
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    motor.usePIDSlot(PIDSlot.SLOT0);
    motor.pid0.setP(config.p);
    motor.pid0.setI(config.i);
    motor.pid0.setD(config.d);
    motor.pid0.setFF(config.f);
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
    //not supported
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    motor.setBrakeMode(isBrakeMode);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    //    Timer.delay(1);
    motor.setInverted(inverted);
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash()
  {
    // Do nothing
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
    if(isDriveMotor){
      motor.setVelocity(setpoint);
      motor.pid0.setFF(feedforward);
    }
    else{
      motor.setPosition(setpoint);
      motor.pid0.setFF(feedforward);
    }
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
    if(isDriveMotor){
      motor.setVelocity(setpoint);
      motor.pid0.setFF(feedforward);
    }
    else{
      motor.setPosition(setpoint);
      motor.pid0.setFF(feedforward);
    }
  }

  /**
   * Get the voltage output of the motor controller.
   *
   * @return Voltage output.
   */
  @Override
  public double getVoltage()
  {
    //not supported
    return 0;
  }

  /**
   * Set the voltage of the motor.
   *
   * @param voltage Voltage to set.
   */
  @Override
  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
  }

  /**
   * Get the applied dutycycle output.
   *
   * @return Applied dutycycle output to the motor.
   */
  @Override
  public double getAppliedOutput()
  { 
    //Not supported
    return 0;
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity in Meters Per Second, or Degrees per Second.
   */
  @Override
  public double getVelocity()
  {
    return motor.getVelocity();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position in Meters or Degrees.
   */
  @Override
  public double getPosition()
  {
    return motor.getPosition();
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position. Should be angle in degrees or meters.
   */
  @Override
  public void setPosition(double position)
  {
    if (!absoluteEncoder && !SwerveDriveTelemetry.isSimulation)
    {
      position = position < 0 ? (position % 360) + 360 : position;
      motor.setEncoderPosition(position);
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
    // Do not implement
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
    motor.setMaxCurrent(CurrentType.STATOR, currentLimit);
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    motor.setRampUp(rampRate);
    motor.setRampDown(rampRate);
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

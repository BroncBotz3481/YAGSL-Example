package frc.robot.subsystems.swervedrive2.parser;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.Drivebase.DriveFeedforwardGains;
import frc.robot.Constants.Drivebase.DrivetrainLimitations;
import frc.robot.Constants.Drivebase.ModulePIDFGains;
import frc.robot.subsystems.swervedrive2.encoders.CANCoderSwerve;
import frc.robot.subsystems.swervedrive2.encoders.SwerveAbsoluteEncoder;
import frc.robot.subsystems.swervedrive2.motors.SparkMaxSwerve;
import frc.robot.subsystems.swervedrive2.motors.SwerveMotor;

public class SwerveModuleConfiguration
{

  public final int     driveMotorID;
  public final int     angleMotorID;
  public final int     absoluteEncoderID;
  public final double  angleOffset;
  public final boolean absoluteEncoderInverted;
  public final boolean driveMotorInverted;
  public final double  maxSpeed = DrivetrainLimitations.MAX_SPEED;
  public final String  driveMotorCANBus, angleMotorCANBus, absoluteEncoderCANBus;
  public double     angleKZ      = ModulePIDFGains.MODULE_KV;
  public PIDFConfig anglePIDF    = new PIDFConfig(ModulePIDFGains.MODULE_KP, ModulePIDFGains.MODULE_KI,
                                                  ModulePIDFGains.MODULE_KD, ModulePIDFGains.MODULE_KF,
                                                  ModulePIDFGains.MODULE_IZ);
  public PIDFConfig velocityPIDF = new PIDFConfig(ModulePIDFGains.VELOCITY_KP, ModulePIDFGains.VELOCITY_KI,
                                                  ModulePIDFGains.VELOCITY_KD, ModulePIDFGains.VELOCITY_KF,
                                                  ModulePIDFGains.VELOCITY_IZ);

  /**
   * Construct a configuration object for swerve modules.
   *
   * @param driveMotorID            Drive motor CAN ID or pin ID.
   * @param angleMotorID            Angle motor CAN ID or pin ID.
   * @param absoluteEncoderID       Absolute encoder CAN ID or pin ID.
   * @param angleOffset             Absolute angle offset to 0.
   * @param absoluteEncoderInverted Absolute encoder inverted.
   * @param driveMotorInverted      Drive motor inverted.
   * @param absoluteEncoderCANBus   CAN Bus name where absolute encoder is attached.
   * @param angleMotorCANBus        CAN Bus name where the angle motor is attached.
   * @param driveMotorCANBus        CAN Bus name where the drive motor is attached.
   */
  public SwerveModuleConfiguration(int driveMotorID, int angleMotorID, int absoluteEncoderID, double angleOffset,
                                   boolean absoluteEncoderInverted, boolean driveMotorInverted, String driveMotorCANBus,
                                   String angleMotorCANBus, String absoluteEncoderCANBus)
  {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.absoluteEncoderID = absoluteEncoderID;
    this.angleOffset = angleOffset;
    this.absoluteEncoderInverted = absoluteEncoderInverted;
    this.driveMotorInverted = driveMotorInverted;
    this.driveMotorCANBus = driveMotorCANBus;
    this.angleMotorCANBus = angleMotorCANBus;
    this.absoluteEncoderCANBus = absoluteEncoderCANBus;
  }

  /**
   * Construct a configuration object for swerve modules.
   *
   * @param driveMotorID            Drive motor CAN ID or pin ID.
   * @param angleMotorID            Angle motor CAN ID or pin ID.
   * @param absoluteEncoderID       Absolute encoder CAN ID or pin ID.
   * @param angleOffset             Absolute angle offset to 0.
   * @param absoluteEncoderInverted Absolute encoder inverted.
   * @param driveMotorInverted      Drive motor inverted.
   */
  public SwerveModuleConfiguration(int driveMotorID, int angleMotorID, int absoluteEncoderID, double angleOffset,
                                   boolean absoluteEncoderInverted, boolean driveMotorInverted)
  {
    this(driveMotorID, angleMotorID, absoluteEncoderID, angleOffset, absoluteEncoderInverted, driveMotorInverted, null,
         null, null);
  }

  /**
   * Construct a configuration object for swerve modules.
   *
   * @param driveMotorID      Drive motor CAN ID or pin ID.
   * @param angleMotorID      Angle motor CAN ID or pin ID.
   * @param absoluteEncoderID Absolute encoder CAN ID or pin ID.
   * @param angleOffset       Absolute angle offset to 0.
   */
  public SwerveModuleConfiguration(int driveMotorID, int angleMotorID, int absoluteEncoderID, double angleOffset)
  {
    this(driveMotorID, angleMotorID, absoluteEncoderID, angleOffset, false, false, null, null, null);
  }

  /**
   * Create the drive feedforward for swerve modules.
   *
   * @return Drive feedforward for drive motor on a swerve module.
   */
  public SimpleMotorFeedforward createDriveFeedforward()
  {
    return new SimpleMotorFeedforward(DriveFeedforwardGains.KS, DriveFeedforwardGains.KV, DriveFeedforwardGains.KA);
  }

  /**
   * Create the {@link SwerveMotor} for the drive motor.
   *
   * @return Instantiated SwerveMotor.
   */
  public SwerveMotor createDriveMotor()
  {
    return new SparkMaxSwerve(driveMotorID, true);
  }

  /**
   * Create the {@link SwerveMotor} for the drive motor.
   *
   * @return Instantiated SwerveMotor.
   */
  public SwerveMotor createAngleMotor()
  {
    return new SparkMaxSwerve(angleMotorID, false);
  }

  /**
   * Create the {@link SwerveAbsoluteEncoder} for the given configuration.
   *
   * @return Instantiated Absolute encoder.
   */
  public SwerveAbsoluteEncoder createAbsoluteEncoder()
  {
    return new CANCoderSwerve(absoluteEncoderID);
  }
}
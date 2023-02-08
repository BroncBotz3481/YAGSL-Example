package frc.robot.subsystems.swervedrive2.parser;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.Drivebase.DriveFeedforwardGains;
import frc.robot.Constants.Drivebase.DrivetrainLimitations;
import frc.robot.Constants.Drivebase.ModulePIDFGains;

public class SwerveModuleConfiguration
{

  public final int        driveMotorID;
  public final int        angleMotorID;
  public final int        cancoderID;
  public final double     angleOffset;
  public final boolean    absoluteEncoderInverted;
  public final boolean    driveMotorInverted;
  public final double     maxSpeed     = DrivetrainLimitations.MAX_SPEED;
  public       double     angleKZ      = ModulePIDFGains.MODULE_KV;
  public       PIDFConfig anglePIDF    = new PIDFConfig(ModulePIDFGains.MODULE_KP, ModulePIDFGains.MODULE_KI,
                                                        ModulePIDFGains.MODULE_KD, ModulePIDFGains.MODULE_KF,
                                                        ModulePIDFGains.MODULE_IZ);
  public       PIDFConfig velocityPIDF = new PIDFConfig(ModulePIDFGains.VELOCITY_KP, ModulePIDFGains.VELOCITY_KI,
                                                        ModulePIDFGains.VELOCITY_KD, ModulePIDFGains.VELOCITY_KF,
                                                        ModulePIDFGains.VELOCITY_IZ);


  public SwerveModuleConfiguration(int driveMotorID, int angleMotorID, int cancoderID, double angleOffset,
                                   boolean absoluteEncoderInverted, boolean driveMotorInverted)
  {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = cancoderID;
    this.angleOffset = angleOffset;
    this.absoluteEncoderInverted = absoluteEncoderInverted;
    this.driveMotorInverted = driveMotorInverted;
  }

  public SwerveModuleConfiguration(int driveMotorID, int angleMotorID, int cancoderID, double angleOffset)
  {
    this(driveMotorID, angleMotorID, cancoderID, angleOffset, false, false);
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
}

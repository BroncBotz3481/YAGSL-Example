package frc.robot.subsystems.swervedrive2.parser;

import frc.robot.Constants.Drivebase.DrivetrainLimitations;
import frc.robot.Constants.Drivebase.ModulePIDFGains;

public class SwerveModuleConfiguration
{

  public final int     driveMotorID;
  public final int     angleMotorID;
  public final int     cancoderID;
  public final double  angleOffset;
  public final boolean absoluteEncoderInverted;
  public final boolean driveMotorInverted;
  public final double  maxSpeed = DrivetrainLimitations.MAX_SPEED;
  public       double  angleKP  = ModulePIDFGains.MODULE_KP, angleKI = ModulePIDFGains.MODULE_KI, angleKD = ModulePIDFGains.MODULE_KD, angleKF = ModulePIDFGains.MODULE_KF, angleKIZ = ModulePIDFGains.MODULE_IZ, angleKZ = ModulePIDFGains.MODULE_KV,
      velocityKP                = ModulePIDFGains.VELOCITY_KP, velocityKI = ModulePIDFGains.VELOCITY_KI, velocityKD = ModulePIDFGains.VELOCITY_KD, velocityKF = ModulePIDFGains.VELOCITY_KF, velocityKIZ = ModulePIDFGains.VELOCITY_IZ;

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
}

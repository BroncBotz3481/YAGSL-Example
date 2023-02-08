package frc.robot.subsystems.swervedrive2.parser;

public class SwerveModuleConstants
{

  public final int     driveMotorID;
  public final int     angleMotorID;
  public final int     cancoderID;
  public final double  angleOffset;
  public final boolean absoluteEncoderInverted;

  public SwerveModuleConstants(int driveMotorID, int angleMotorID, int cancoderID, double angleOffset,
                               boolean absoluteEncoderInverted)
  {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = cancoderID;
    this.angleOffset = angleOffset;
    this.absoluteEncoderInverted = absoluteEncoderInverted;
  }

  public SwerveModuleConstants(int driveMotorID, int angleMotorID, int cancoderID, double angleOffset)
  {
    this(driveMotorID, angleMotorID, cancoderID, angleOffset, false);
  }
}

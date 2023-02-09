package frc.robot.subsystems.swervedrive2.swervelib.parser;

import static frc.robot.subsystems.swervedrive2.swervelib.math.SwerveMath.calculateMaxAngularVelocity;

public class SwerveControllerConfiguration
{

  /**
   * Maximum robot speed in meters per second.
   */
  public final double     maxSpeed;
  /**
   * Maximum angular velocity in rad/s
   */
  public final double     maxAngularVelocity;
  /**
   * PIDF for the heading of the robot.
   */
  public final PIDFConfig headingPIDF;
  /**
   * hypotenuse deadband for the robot angle control joystick.
   */
  public final double     hypotDeadband; // Deadband for the minimum hypot for the heading joystick.

  /**
   * Construct the swerve controller configuration.
   *
   * @param driveCfg      Drive configuration.
   * @param headingPIDF   Heading PIDF configuration.
   * @param hypotDeadband Deadband on radius of angle joystick.
   */
  public SwerveControllerConfiguration(SwerveDriveConfiguration driveCfg, PIDFConfig headingPIDF, double hypotDeadband)
  {
    this.maxSpeed = driveCfg.maxSpeed;
    this.maxAngularVelocity = calculateMaxAngularVelocity(driveCfg.maxSpeed,
                                                          driveCfg.moduleLocationsMeters[0].getX(),
                                                          driveCfg.moduleLocationsMeters[0].getY());
    this.headingPIDF = headingPIDF;
    this.hypotDeadband = hypotDeadband;
  }

  /**
   * Construct the swerve controller configuration. Assumes hypotenuse deadband of 0.5 (minimum radius for angle to be
   * set on angle joystick is .5 of the controller).
   *
   * @param driveCfg    Drive configuration.
   * @param headingPIDF Heading PIDF configuration.
   */
  public SwerveControllerConfiguration(SwerveDriveConfiguration driveCfg, PIDFConfig headingPIDF)
  {
    this(driveCfg, headingPIDF, 0.5);
  }

}

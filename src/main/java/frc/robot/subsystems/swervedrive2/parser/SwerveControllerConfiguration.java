package frc.robot.subsystems.swervedrive2.parser;

import static frc.robot.subsystems.swervedrive2.math.SwerveMath.calculateMaxAngularVelocity;

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
  public       double     hypotDeadband = 0.5; // Deadband for the minimum hypot for the heading joystick.

  /**
   * Construct the swerve controller configuration.
   *
   * @param driveCfg    Drive configuration.
   * @param maxSpeed    Maximum robot speed in meters per second.
   * @param headingPIDF Heading PIDF configuration.
   */
  public SwerveControllerConfiguration(SwerveDriveConfiguration driveCfg, double maxSpeed, PIDFConfig headingPIDF)
  {
    this.maxSpeed = maxSpeed;
    this.maxAngularVelocity = calculateMaxAngularVelocity(maxSpeed,
                                                          driveCfg.moduleLocationsMeters[0].getX(),
                                                          driveCfg.moduleLocationsMeters[0].getY());
    this.headingPIDF = headingPIDF;
  }

}

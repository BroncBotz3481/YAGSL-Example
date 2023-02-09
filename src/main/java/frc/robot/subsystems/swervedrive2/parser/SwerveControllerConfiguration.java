package frc.robot.subsystems.swervedrive2.parser;

import static frc.robot.subsystems.swervedrive2.math.SwerveMath.calculateMaxAngularVelocity;

public class SwerveControllerConfiguration
{

  public final double     maxSpeed;
  public final double     maxAngularVelocity;
  public final PIDFConfig headingPIDF;
  public       double     hypotDeadband = 0.5; // Deadband for the minimum hypot for the heading joystick.

  public SwerveControllerConfiguration(SwerveDriveConfiguration driveCfg, double maxSpeed, PIDFConfig headingPIDF)
  {
    this.maxSpeed = maxSpeed;
    this.maxAngularVelocity = calculateMaxAngularVelocity(maxSpeed,
                                                          driveCfg.moduleLocationsMeters[0].getX(),
                                                          driveCfg.moduleLocationsMeters[0].getY());
    this.headingPIDF = headingPIDF;
  }

}

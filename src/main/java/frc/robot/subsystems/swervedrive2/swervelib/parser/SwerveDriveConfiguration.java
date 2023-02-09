package frc.robot.subsystems.swervedrive2.swervelib.parser;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swervedrive2.swervelib.SwerveModule;
import frc.robot.subsystems.swervedrive2.swervelib.imu.SwerveIMU;
import java.util.ArrayList;
import java.util.List;

/**
 * Swerve drive configurations used during SwerveDrive construction.
 */
public class SwerveDriveConfiguration
{

  /**
   * Swerve Module locations.
   */
  public Translation2d[] moduleLocationsMeters;
  /**
   * Swerve IMU
   */
  public SwerveIMU       imu;
  /**
   * Max speed in meters per second.
   */
  public double          maxSpeed;
  /**
   * Number of modules on the robot.
   */
  public int             moduleCount;
  /**
   * Swerve Modules.
   */
  public SwerveModule[]  modules;

  /**
   * Create swerve drive configuration.
   *
   * @param moduleConfigs Module configuration.
   * @param swerveIMU     Swerve IMU.
   * @param maxSpeed      Max speed of the robot in meters per second.
   */
  public SwerveDriveConfiguration(SwerveModuleConfiguration[] moduleConfigs, SwerveIMU swerveIMU, double maxSpeed)
  {
    this.moduleCount = moduleConfigs.length;
    this.imu = swerveIMU;
    this.maxSpeed = maxSpeed;
    this.modules = createModules(moduleConfigs);
    this.moduleLocationsMeters = new Translation2d[moduleConfigs.length];
    for (SwerveModule module : modules)
    {
      this.moduleLocationsMeters[module.moduleNumber] = module.configuration.moduleLocation;
    }
  }

  /**
   * Create modules based off of the SwerveModuleConfiguration.
   *
   * @param swerves Swerve constants.
   * @return Swerve Modules.
   */
  public SwerveModule[] createModules(SwerveModuleConfiguration[] swerves)
  {
    List<SwerveModule> mods   = new ArrayList<>();
    SwerveModule[]     modArr = new SwerveModule[moduleCount];

    for (int i = 0; i < swerves.length; i++)
    {
      mods.add(new SwerveModule(i, swerves[i]));
    }
    return mods.toArray(modArr);
  }
}

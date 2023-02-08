package frc.robot.subsystems.swervedrive2.parser;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.Drivebase.DrivetrainLimitations;
import frc.robot.Constants.Drivebase.Mod0FL;
import frc.robot.Constants.Drivebase.Mod1FR;
import frc.robot.Constants.Drivebase.Mod2BL;
import frc.robot.Constants.Drivebase.Mod3BR;
import frc.robot.Constants.Drivebase.ModuleLocations;
import frc.robot.subsystems.swervedrive2.SwerveModule;
import frc.robot.subsystems.swervedrive2.imu.PigeonSwerve;
import frc.robot.subsystems.swervedrive2.imu.SwerveIMU;
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
  public Translation2d[] moduleLocationsMeters = new Translation2d[]{
      new Translation2d(ModuleLocations.FRONT_LEFT_X, ModuleLocations.FRONT_LEFT_Y),
      new Translation2d(ModuleLocations.FRONT_RIGHT_X, ModuleLocations.FRONT_RIGHT_Y),
      new Translation2d(ModuleLocations.BACK_LEFT_X, ModuleLocations.BACK_LEFT_Y),
      new Translation2d(ModuleLocations.BACK_RIGHT_X, ModuleLocations.BACK_RIGHT_Y)};
  /**
   * Swerve IMU
   */
  public SwerveIMU       imu                   = new PigeonSwerve(Drivebase.PIGEON);
  /**
   * Max speed in meters per second.
   */
  public double          maxSpeed              = DrivetrainLimitations.MAX_SPEED;
  /**
   * Number of modules on the robot.
   */
  public int             moduleCount           = Drivebase.NUM_MODULES;
  /**
   * Swerve Modules.
   */
  public SwerveModule[]  modules               = createModules(new SwerveModuleConfiguration[]{Mod0FL.CONSTANTS,
                                                                                               Mod1FR.CONSTANTS,
                                                                                               Mod2BL.CONSTANTS,
                                                                                               Mod3BR.CONSTANTS});


  /**
   * Create modules based off of the SwerveModuleConfiguration.
   *
   * @param swerves Swerve constants.
   * @return Swerve Modules.
   */
  public SwerveModule[] createModules(SwerveModuleConfiguration[] swerves)
  {
    List<SwerveModule> mods   = new ArrayList<>();
    SwerveModule[]     modArr = new SwerveModule[swerves.length];

    for (int i = 0; i < swerves.length; i++)
    {
      mods.add(new SwerveModule(i, swerves[i]));
    }
    return mods.toArray(modArr);
  }
}

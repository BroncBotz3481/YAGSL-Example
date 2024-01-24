package swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Class to perform tests on the swerve drive.
 */
public class SwerveDriveTest
{

  /**
   * Set the modules to center to 0.
   *
   * @param swerveDrive Swerve Drive to control.
   */
  public static void centerModules(SwerveDrive swerveDrive)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false, true);

    }

    // Update kinematics because we are not using setModuleStates
    swerveDrive.kinematics.toSwerveModuleStates(new ChassisSpeeds());
  }

  /**
   * Power the drive motors for the swerve drive to a set percentage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param percentage  Percentage of voltage to send to drive motors.
   */
  public static void powerDriveMotors(SwerveDrive swerveDrive, double percentage)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getDriveMotor().set(percentage);
    }
  }

  /**
   * Power the angle motors for the swerve drive to a set percentage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param percentage  Percentage of voltage to send to angle motors.
   */
  public static void powerAngleMotors(SwerveDrive swerveDrive, double percentage)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getAngleMotor().set(percentage);
    }
  }

  /**
   * Find the minimum amount of power required to move the swerve drive motors.
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param minMovement Minimum amount of movement to drive motors.
   * @return minimum voltage required.
   */
  public static double findDriveMotorKV(SwerveDrive swerveDrive, double minMovement)
  {
    double[] startingEncoders = new double[4];
    double   kV               = 0;

    SwerveDriveTest.powerDriveMotors(swerveDrive, 0);
    SwerveModule[] modules = swerveDrive.getModules();
    for (int i = 0; i < modules.length; i++)
    {
      startingEncoders[i] = Math.abs(modules[i].getDriveMotor().getPosition());
    }

    for(double kV_new = 0; kV_new < 0.1; kV_new += 0.0001)
    {
      SwerveDriveTest.powerDriveMotors(swerveDrive, kV);
      boolean foundkV = false;
      for (int i = 0; i < modules.length; i++)
      {
        if((modules[i].getDriveMotor().getPosition() - startingEncoders[i]) > minMovement)
        {
          foundkV = true;
          break;
        }
      }
      if(foundkV)
      {
        SwerveDriveTest.powerDriveMotors(swerveDrive, 0);
        kV = kV_new;
      }
    }
    return kV;
  }
}

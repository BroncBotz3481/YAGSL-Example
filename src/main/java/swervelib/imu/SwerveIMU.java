package swervelib.imu;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Swerve IMU abstraction to define a standard interface with a swerve drive.
 */
public abstract class SwerveIMU
{

  /**
   * Cache timeout in milliseconds. Data will refresh after this many milliseconds.
   */
  private final double   cachedTimeout   = 5;
  /**
   * Cached yaw/pitch/roll array.
   */
  private final double[] cachedYpr       = new double[3];
  /**
   * Time last cache was taken in microseconds.
   */
  private       double   cachedTimeMicro = 0;

  /**
   * Reset IMU to factory default.
   */
  public abstract void factoryDefault();

  /**
   * Clear sticky faults on IMU.
   */
  public abstract void clearStickyFaults();

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Yaw angle in degrees.
   */
  public abstract void setYaw(double yaw);

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  public abstract void getYawPitchRoll(double[] yprArray);

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  public void getCachedYawPitchRoll(double[] yprArray)
  {
    if ((RobotController.getFPGATime() - cachedTimeMicro) < (cachedTimeout * 1000))
    {
      yprArray[0] = cachedYpr[0];
      yprArray[1] = cachedYpr[1];
      yprArray[2] = cachedYpr[2];
    } else
    {
      getYawPitchRoll(yprArray);
      cachedYpr[0] = yprArray[0];
      cachedYpr[1] = yprArray[1];
      cachedYpr[2] = yprArray[2];
      cachedTimeMicro = RobotController.getFPGATime();
    }
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  public abstract Object getIMU();
}

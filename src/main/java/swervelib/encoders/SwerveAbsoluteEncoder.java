package swervelib.encoders;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Swerve abstraction class to define a standard interface with absolute encoders for swerve modules..
 */
public abstract class SwerveAbsoluteEncoder
{

  /**
   * Cache timeout in milliseconds. Data will refresh after this many milliseconds.
   */
  private final double  cachedTimeout   = 5;
  /**
   * Last angle reading was faulty.
   */
  public        boolean readingError    = false;
  /**
   * Time last cache was taken in microseconds.
   */
  private       double  cachedTimeMicro = RobotController.getFPGATime();
  /**
   * The cached absolute encoder position.
   */
  private       double  cachedPosition  = 0;

  /**
   * Reset the encoder to factory defaults.
   */
  public abstract void factoryDefault();

  /**
   * Clear sticky faults on the encoder.
   */
  public abstract void clearStickyFaults();

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  public abstract void configure(boolean inverted);

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  public abstract double getAbsolutePosition();

  /**
   * Get the absolute position of the encoder from the cache if available.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  public double getCachedAbsolutePosition()
  {
    if ((RobotController.getFPGATime() - cachedTimeMicro) > (cachedTimeout * 1000))
    {
      cachedTimeMicro = RobotController.getFPGATime();
      cachedPosition = getAbsolutePosition();
    }
    return cachedPosition;
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  public abstract Object getAbsoluteEncoder();
}

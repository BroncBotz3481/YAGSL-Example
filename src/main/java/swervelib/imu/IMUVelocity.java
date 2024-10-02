package swervelib.imu;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

public class IMUVelocity {
  /**
   * Swerve IMU.
   */
  private final SwerveIMU gyro;
  /**
   * Linear filter used to calculate velocity.
   */
  private final LinearFilter velocityFilter;
  /**
   * WPILib {@link Notifier} to keep IMU velocity up to date.
   */
  private final Notifier  notifier;

  /**
   * Prevents calculation when no previous measurement exists.
   */
  private boolean firstCycle = true;
  /**
   * Tracks the previous loop's recorded time.
   */
  private double timestamp = 0.0;
  /**
   * Tracks the previous loop's position as a Rotation2d. 
   */
  private Rotation2d position = new Rotation2d();
  /**
   * The calculated velocity of the robot based on averaged IMU measurements.
   */
  private double velocity = 0.0;

  /**
   * Constructor for the IMU Velocity. Defaults to an IMU polling rate of 50hz 
   * and 5 taps for moving average linear filter.
   *
   * @param gyro The SwerveIMU gyro.
   */
  public IMUVelocity(SwerveIMU gyro)
  {
    /* TODO: For now I am going to assume that gyros operate at 60hz or more by default  */
    /* NAVX2.0 defaults to 60hz, Pigeon2 is 100hz  */
    this(gyro, 1.0/60.0, 5);
  }

  /**
   * Constructor for the IMU Velocity.
   *
   * @param gyro The SwerveIMU gyro.
   * @param periodSeconds The rate to collect measurements from the gyro, in the form (1/number of samples per second),
   * make sure this does not exceed the update rate of your IMU.
   * @param averagingTaps The number of samples to used for the moving average linear filter. Higher values will not
   * allow the system to update to changes in velocity, lower values may create a less smooth signal. Expected taps
   * will probably be ~2-8, with the goal of having the lowest smooth value.
   * 
   */
  public IMUVelocity(SwerveIMU gyro, double periodSeconds, int averagingTaps)
  {
    this.gyro = gyro;
    velocityFilter = LinearFilter.movingAverage(averagingTaps);
    notifier = new Notifier(this::update);
    notifier.startPeriodic(periodSeconds);
    timestamp = RobotController.getFPGATime();
  }

  /**
   * Update the robot's rotational velocity based on the current gyro position.
   */
  private void update() 
  {
    double newTimestamp = RobotController.getFPGATime();
    Rotation2d newPosition = Rotation2d.fromRadians(gyro.getRotation3d().getZ());

    synchronized (this) {
      if (!firstCycle) {
        velocity = velocityFilter.calculate(
            (newPosition.minus(position).getRadians()) / (newTimestamp - timestamp));
        }
      firstCycle = false;
      timestamp = newTimestamp;
      position = newPosition;
    }
  }

  /**
   * Get the robot's angular velocity based on averaged meaasurements from the IMU.
   *
   * @return robot's angular velocity in rads/s as a double.
   */
  public synchronized double getVelocity() {
    return velocity;
  }
}

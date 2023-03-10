package swervelib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Second order kinematics swerve module state. */
public class SwerveModuleState2 extends SwerveModuleState {

  /** Swerve module position in meters. */
  public double distanceMeters = 0;
  /** Swerve module speed in meters per second */
  public double speedMetersPerSecond = 0;
  /** Swerve module acceleration in meters per second squared */
  public double accelMetersPerSecondSq = 0;
  /** Rad per sec */
  public double omegaRadPerSecond = 0;
  /** Swerve module angle as a {@link Rotation2d}. */
  public Rotation2d angle = Rotation2d.fromDegrees(0);

  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public SwerveModuleState2() {}

  /**
   * Constructs a SwerveModuleState.
   *
   * @param distanceMeters The speed of the wheel of the module.
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param accelMetersPerSecondSq The speed of the wheel of the module.
   * @param angle The angle of the module.
   * @param omegaRadPerSecond The angular velocity of the module.
   */
  public SwerveModuleState2(
      double distanceMeters, double speedMetersPerSecond, double accelMetersPerSecondSq, Rotation2d angle, double omegaRadPerSecond) {
    this.distanceMeters = distanceMeters;
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.accelMetersPerSecondSq = accelMetersPerSecondSq;
    this.angle = angle;
    this.omegaRadPerSecond = omegaRadPerSecond;
  }

  @Override
  public String toString() {
    return String.format(
            "SwerveModuleState(Position: %.2f m, Speed: %.2f m/s, Accel: %.2f m/s/s, Angle: %s, Omega: %.2f rad/s)", distanceMeters, speedMetersPerSecond, accelMetersPerSecondSq, angle, omegaRadPerSecond);
  }
}

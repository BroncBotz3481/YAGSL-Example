package swervelib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Vector containing Magnitude and Position
 */
public class Vector2d
{

  /**
   * Position given as an angle {@link Rotation2d}.
   */
  public final Rotation2d position;
  /**
   * Arbitrary magnitude of the vector.
   */
  public final double     magnitude;

  /**
   * Construct a Vector with a position and magnitude of 0.
   */
  public Vector2d()
  {
    position = new Rotation2d();
    magnitude = 0;
  }

  /**
   * Create a vector based off of the given {@link Translation2d}. The magnitude is the
   * {@link Math#hypot(double, double)} of the X and Y.
   *
   * @param cartesian {@link Translation2d} with the X and Y set.
   */
  public Vector2d(Translation2d cartesian)
  {
    position = cartesian.getAngle();
    magnitude = cartesian.getNorm();
  }

  /**
   * Construct a {@link Vector2d} given the position and magnitude.
   *
   * @param position  Position as a {@link Rotation2d}.
   * @param magnitude Magnitude as a double.
   */
  public Vector2d(Rotation2d position, double magnitude)
  {
    this.position = position;
    this.magnitude = magnitude;
  }

  /**
   * Convert cartesian coordinates to Vector.
   *
   * @param x X position
   * @param y Y position
   */
  public Vector2d(double x, double y)
  {
    this(new Translation2d(x, y));
  }

  /**
   * Convert the Vector back into cartesian coordinates.
   *
   * @return {@link Translation2d} of the vector.
   */
  public Translation2d getTranslation()
  {
    return new Translation2d(magnitude, position);
  }

  /**
   * Scale the magnitude by the multiplier given
   *
   * @param scalar Multiplier of the magnitude.
   * @return {@link Vector2d} for chained functions.
   */
  public Vector2d scale(double scalar)
  {
    return new Vector2d(position, magnitude * scalar);
  }

  /**
   * Exponentially modify the magnitude.
   *
   * @param pow Power for the magnitude
   * @return {@link Vector2d} with the magnitude^pow
   */
  public Vector2d pow(double pow)
  {
    return new Vector2d(position, Math.pow(this.magnitude, pow));
  }
}

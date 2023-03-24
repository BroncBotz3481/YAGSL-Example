package webblib.util.swerve;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import webblib.math.BetterMath;

public class SwerveBalance {
  private final double scale;
  private final double scalePow;

  /**
   * Initialize SwerveBalance class.
   *
   * @param scale The scalar to apply to the gradients.
   * @param scalePow Weight the result to be nonlinear (faster to balance when farther away). Set to
   *     1 to be linear. Must be greater than 0.
   */
  public SwerveBalance(double scale, double scalePow) {
    if (scalePow <= 0) {
      throw new IllegalArgumentException("scalePow must be greater than 0");
    }
    this.scale = scale;
    this.scalePow = scalePow;
  }

  /**
   * Update robot speeds for balancing based on pitch and roll. Creates a plane which the robot sits
   * on to update its speeds accordingly. Tunable with contructor.
   *
   * @param gyroAngle The angle of the gyro as a {@link Rotation3d}.
   * @return {@link Translation2d} object for use with {@link
   *     edu.wpi.first.math.kinematics.ChassisSpeeds}.
   */
  public Translation2d calculate(Rotation3d angle) {

    var xGrad = Math.tan(angle.getY());
    var yGrad = -Math.tan(angle.getX());

    var vyMetersPerSecond =
        BetterMath.signedAbsFunc(yGrad, (x) -> Math.pow(Math.abs(x * scale), scalePow));
    var vxMetersPerSecond =
        BetterMath.signedAbsFunc(xGrad, (x) -> Math.pow(Math.abs(x * scale), scalePow));

    return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
  }
}

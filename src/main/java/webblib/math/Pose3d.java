package webblib.math;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Objects;

/** Represents a 3D pose containing translational and rotational elements. */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public class Pose3d implements Interpolatable<Pose3d> {
  private final Translation3d m_translation;
  private final Rotation3d m_rotation;

  /** Constructs a pose at the origin facing toward the positive X axis. */
  public Pose3d() {
    m_translation = new Translation3d();
    m_rotation = new Rotation3d();
  }

  /**
   * Constructs a pose with the specified translation and rotation.
   *
   * @param translation The translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  @JsonCreator
  public Pose3d(
      @JsonProperty(required = true, value = "translation") Translation3d translation,
      @JsonProperty(required = true, value = "rotation") Rotation3d rotation) {
    m_translation = translation;
    m_rotation = rotation;
  }

  /**
   * Constructs a pose with x, y, and z translations instead of a separate Translation3d.
   *
   * @param x The x component of the translational component of the pose.
   * @param y The y component of the translational component of the pose.
   * @param z The z component of the translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  public Pose3d(double x, double y, double z, Rotation3d rotation) {
    m_translation = new Translation3d(x, y, z);
    m_rotation = rotation;
  }

  /**
   * Constructs a 3D pose from a 2D pose in the X-Y plane.
   *
   * @param pose The 2D pose.
   */
  public Pose3d(Pose2d pose) {
    m_translation = new Translation3d(pose.getX(), pose.getY(), 0.0);
    m_rotation = new Rotation3d(0.0, 0.0, pose.getRotation().getRadians());
  }

  /**
   * Transforms the pose by the given transformation and returns the new transformed pose.
   *
   * @param other The transform to transform the pose by.
   * @return The transformed pose.
   */
  public Pose3d plus(Transform3d other) {
    return transformBy(other);
  }

  /**
   * Returns the Transform3d that maps the one pose to another.
   *
   * @param other The initial pose of the transformation.
   * @return The transform that maps the other pose to the current pose.
   */
  public Transform3d minus(Pose3d other) {
    final var pose = this.relativeTo(other);
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Returns the translation component of the transformation.
   *
   * @return The translational component of the pose.
   */
  @JsonProperty
  public Translation3d getTranslation() {
    return m_translation;
  }

  /**
   * Returns the X component of the pose's translation.
   *
   * @return The x component of the pose's translation.
   */
  public double getX() {
    return m_translation.getX();
  }

  /**
   * Returns the Y component of the pose's translation.
   *
   * @return The y component of the pose's translation.
   */
  public double getY() {
    return m_translation.getY();
  }

  /**
   * Returns the Z component of the pose's translation.
   *
   * @return The z component of the pose's translation.
   */
  public double getZ() {
    return m_translation.getZ();
  }

  /**
   * Returns the rotational component of the transformation.
   *
   * @return The rotational component of the pose.
   */
  @JsonProperty
  public Rotation3d getRotation() {
    return m_rotation;
  }

  /**
   * Multiplies the current pose by a scalar.
   *
   * @param scalar The scalar.
   * @return The new scaled Pose3d.
   */
  public Pose3d times(double scalar) {
    return new Pose3d(m_translation.times(scalar), m_rotation.times(scalar));
  }

  /**
   * Divides the current pose by a scalar.
   *
   * @param scalar The scalar.
   * @return The new scaled Pose3d.
   */
  public Pose3d div(double scalar) {
    return times(1.0 / scalar);
  }

  /**
   * Transforms the pose by the given transformation and returns the new pose. See + operator for
   * the matrix multiplication performed.
   *
   * @param other The transform to transform the pose by.
   * @return The transformed pose.
   */
  public Pose3d transformBy(Transform3d other) {
    return new Pose3d(
        m_translation.plus(other.getTranslation().rotateBy(m_rotation)),
        other.getRotation().plus(m_rotation));
  }

  /**
   * Returns the current pose relative to the given pose.
   *
   * <p>This function can often be used for trajectory tracking or pose stabilization algorithms to
   * get the error between the reference and the current pose.
   *
   * @param other The pose that is the origin of the new coordinate frame that the current pose will
   *     be converted into.
   * @return The current pose relative to the new origin pose.
   */
  public Pose3d relativeTo(Pose3d other) {
    var transform = new Transform3d(other, this);
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Obtain a new Pose3d from a (constant curvature) velocity.
   *
   * <p>The twist is a change in pose in the robot's coordinate frame since the previous pose
   * update. When the user runs exp() on the previous known field-relative pose with the argument
   * being the twist, the user will receive the new field-relative pose.
   *
   * <p>"Exp" represents the pose exponential, which is solving a differential equation moving the
   * pose forward in time.
   *
   * @param twist The change in pose in the robot's coordinate frame since the previous pose update.
   *     For example, if a non-holonomic robot moves forward 0.01 meters and changes angle by 0.5
   *     degrees since the previous pose update, the twist would be Twist3d(0.01, 0.0, 0.0, new new
   *     Rotation3d(0.0, 0.0, Units.degreesToRadians(0.5))).
   * @return The new pose of the robot.
   */
  public Pose3d exp(Twist3d twist) {
    // Implementation from Section 3.2 of https://ethaneade.org/lie.pdf
    final var u = VecBuilder.fill(twist.dx, twist.dy, twist.dz);
    final var rvec = VecBuilder.fill(twist.rx, twist.ry, twist.rz);
    final var omega = rotationVectorToMatrix(rvec);
    final var omegaSq = omega.times(omega);
    double theta = rvec.norm();
    double thetaSq = theta * theta;

    double A;
    double B;
    double C;
    if (Math.abs(theta) < 1E-8) {
      // Taylor Expansions around θ = 0
      // A = 1/1! - θ²/3! + θ⁴/5!
      // B = 1/2! - θ²/4! + θ⁴/6!
      // C = 1/3! - θ²/5! + θ⁴/7!
      // sources:
      // A:
      // https://www.wolframalpha.com/input?i2d=true&i=series+expansion+of+Divide%5Bsin%5C%2840%29x%5C%2841%29%2Cx%5D+at+x%3D0
      // B:
      // https://www.wolframalpha.com/input?i2d=true&i=series+expansion+of+Divide%5B1-cos%5C%2840%29x%5C%2841%29%2CPower%5Bx%2C2%5D%5D+at+x%3D0
      // C:
      // https://www.wolframalpha.com/input?i2d=true&i=series+expansion+of+Divide%5B1-Divide%5Bsin%5C%2840%29x%5C%2841%29%2Cx%5D%2CPower%5Bx%2C2%5D%5D+at+x%3D0
      A = 1 - thetaSq / 6 + thetaSq * thetaSq / 120;
      B = 1 / 2.0 - thetaSq / 24 + thetaSq * thetaSq / 720;
      C = 1 / 6.0 - thetaSq / 120 + thetaSq * thetaSq / 5040;
    } else {
      // A = sin(θ)/θ
      // B = (1 - cos(θ)) / θ²
      // C = (1 - A) / θ²
      A = Math.sin(theta) / theta;
      B = (1 - Math.cos(theta)) / thetaSq;
      C = (1 - A) / thetaSq;
    }

    Matrix<N3, N3> R = Matrix.eye(Nat.N3()).plus(omega.times(A)).plus(omegaSq.times(B));
    Matrix<N3, N3> V = Matrix.eye(Nat.N3()).plus(omega.times(B)).plus(omegaSq.times(C));
    Matrix<N3, N1> translation_component = V.times(u);
    final var transform =
        new Transform3d(
            new Translation3d(
                translation_component.get(0, 0),
                translation_component.get(1, 0),
                translation_component.get(2, 0)),
            new Rotation3d(R));

    return this.plus(transform);
  }

  /**
   * Returns a Twist3d that maps this pose to the end pose. If c is the output of {@code a.Log(b)},
   * then {@code a.Exp(c)} would yield b.
   *
   * @param end The end pose for the transformation.
   * @return The twist that maps this to end.
   */
  public Twist3d log(Pose3d end) {
    // Implementation from Section 3.2 of https://ethaneade.org/lie.pdf
    final var transform = end.relativeTo(this);

    // System.out.println(transform.toString());

    final var rvec = transform.getRotation().getQuaternion().toRotationVector();

    // System.out.println(rvec.toString());

    final var omega = rotationVectorToMatrix(rvec);
    final var theta = rvec.norm();
    final var thetaSq = theta * theta;

    // System.out.println(theta);

    double C;
    if (Math.abs(theta) < 1E-8) {
      // Taylor Expansions around θ = 0
      // A = 1/1! - θ²/3! + θ⁴/5!
      // B = 1/2! - θ²/4! + θ⁴/6!
      // C = 1/6 * (1/2 + θ²/5! + θ⁴/7!)
      // sources:
      // A:
      // https://www.wolframalpha.com/input?i2d=true&i=series+expansion+of+Divide%5Bsin%5C%2840%29x%5C%2841%29%2Cx%5D+at+x%3D0
      // B:
      // https://www.wolframalpha.com/input?i2d=true&i=series+expansion+of+Divide%5B1-cos%5C%2840%29x%5C%2841%29%2CPower%5Bx%2C2%5D%5D+at+x%3D0
      // C:
      // https://www.wolframalpha.com/input?i2d=true&i=series+expansion+of+Divide%5B1-Divide%5BDivide%5Bsin%5C%2840%29x%5C%2841%29%2Cx%5D%2C2Divide%5B1-cos%5C%2840%29x%5C%2841%29%2CPower%5Bx%2C2%5D%5D%5D%2CPower%5Bx%2C2%5D%5D+at+x%3D0
      C = 1 / 12.0 + thetaSq / 720 + thetaSq * thetaSq / 30240;
    } else {
      // A = sin(θ)/θ
      // B = (1 - cos(θ)) / θ²
      // C = (1 - A/(2*B)) / θ²
      double A = Math.sin(theta) / theta;
      double B = (1 - Math.cos(theta)) / thetaSq;
      C = (1 - A / (2 * B)) / thetaSq;
    }

    final var V_inv =
        Matrix.eye(Nat.N3()).minus(omega.times(0.5)).plus(omega.times(omega).times(C));

    final var twist_translation =
        V_inv.times(VecBuilder.fill(transform.getX(), transform.getY(), transform.getZ()));

    return new Twist3d(
        twist_translation.get(0, 0),
        twist_translation.get(1, 0),
        twist_translation.get(2, 0),
        rvec.get(0, 0),
        rvec.get(1, 0),
        rvec.get(2, 0));
  }

  /**
   * Returns a Pose2d representing this Pose3d projected into the X-Y plane.
   *
   * @return A Pose2d representing this Pose3d projected into the X-Y plane.
   */
  public Pose2d toPose2d() {
    return new Pose2d(m_translation.toTranslation2d(), m_rotation.toRotation2d());
  }

  @Override
  public String toString() {
    return String.format("Pose3d(%s, %s)", m_translation, m_rotation);
  }

  /**
   * Checks equality between this Pose3d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Pose3d) {
      return ((Pose3d) obj).m_translation.equals(m_translation)
          && ((Pose3d) obj).m_rotation.equals(m_rotation);
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_translation, m_rotation);
  }

  @Override
  public Pose3d interpolate(Pose3d endValue, double t) {
    if (t < 0) {
      return this;
    } else if (t >= 1) {
      return endValue;
    } else {
      var twist = this.log(endValue);
      var scaledTwist =
          new Twist3d(
              twist.dx * t, twist.dy * t, twist.dz * t, twist.rx * t, twist.ry * t, twist.rz * t);
      return this.exp(scaledTwist);
    }
  }

  /**
   * Applies the hat operator to a rotation vector.
   *
   * <p>It takes a rotation vector and returns the corresponding matrix representation of the Lie
   * algebra element (a 3x3 rotation matrix).
   *
   * @param rotation The rotation vector.
   * @return The rotation vector as a 3x3 rotation matrix.
   */
  private Matrix<N3, N3> rotationVectorToMatrix(Vector<N3> rotation) {
    // Given a rotation vector <a, b, c>,
    //         [ 0 -c  b]
    // Omega = [ c  0 -a]
    //         [-b  a  0]
    return new MatBuilder<>(Nat.N3(), Nat.N3())
        .fill(
            0.0,
            -rotation.get(2, 0),
            rotation.get(1, 0),
            rotation.get(2, 0),
            0.0,
            -rotation.get(0, 0),
            -rotation.get(1, 0),
            rotation.get(0, 0),
            0.0);
  }
}

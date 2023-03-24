package webblib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmPIDController extends PIDController {

  // Factor for "proportional" control
  private double m_kp;

  // Factor for "integral" control
  private double m_ki;

  // Factor for "derivative" control
  private double m_kd;

  // The period (in seconds) of the loop that calls the controller
  private final double m_period;

  private double m_maximumIntegral = 1.0;

  private double m_minimumIntegral = -1.0;

  // The error at the time of the most recent call to calculate()
  private Rotation2d m_positionError = new Rotation2d();
  private double m_velocityError;

  private Rotation2d m_maximumAvoidanceBound = new Rotation2d();
  private Rotation2d m_minimumAvoidanceBound = new Rotation2d();
  private Rotation2d m_middleAvoidanceBound = new Rotation2d();
  private Rotation2d m_avoidanceTolerance = Rotation2d.fromDegrees(2.0);

  // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
  private Rotation2d m_prevError = new Rotation2d();

  // The sum of the errors for use in the integral calc
  private double m_totalError;

  // The error that is considered at setpoint.
  private double m_positionTolerance = 0.05;
  private double m_velocityTolerance = Double.POSITIVE_INFINITY;

  private Rotation2d m_setpoint;
  private Rotation2d m_measurement;

  private boolean m_haveMeasurement;
  private boolean m_haveSetpoint;

  /**
   * Allocates a new {@link ArmPIDController} with the given constants for kp, ki, and kd and a
   * default period of 0.02 seconds. For use with an absolute encoder and avoidance zone.
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   */
  public ArmPIDController(double kp, double ki, double kd) {
    this(kp, ki, kd, 0.02);
  }

  /**
   * Allocates a PIDController with the given constants for kp, ki, and kd.
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   * @param period The period between controller updates in seconds. Must be non-zero and positive.
   */
  public ArmPIDController(double kp, double ki, double kd, double period) {
    super(kp, ki, kd, period);
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

    if (period <= 0) {
      throw new IllegalArgumentException("Controller period must be a non-zero positive number!");
    }
    m_period = period;
  }

  /**
   * Sets the PID Controller gain parameters.
   *
   * <p>Set the proportional, integral, and differential coefficients.
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   */
  public void setPID(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
  }

  /**
   * Sets the Proportional coefficient of the PID controller gain.
   *
   * @param kp proportional coefficient
   */
  public void setP(double kp) {
    m_kp = kp;
  }

  /**
   * Sets the Integral coefficient of the PID controller gain.
   *
   * @param ki integral coefficient
   */
  public void setI(double ki) {
    m_ki = ki;
  }

  /**
   * Sets the Differential coefficient of the PID controller gain.
   *
   * @param kd differential coefficient
   */
  public void setD(double kd) {
    m_kd = kd;
  }

  /**
   * Get the Proportional coefficient.
   *
   * @return proportional coefficient
   */
  public double getP() {
    return m_kp;
  }

  /**
   * Get the Integral coefficient.
   *
   * @return integral coefficient
   */
  public double getI() {
    return m_ki;
  }

  /**
   * Get the Differential coefficient.
   *
   * @return differential coefficient
   */
  public double getD() {
    return m_kd;
  }

  /**
   * Returns the period of this controller.
   *
   * @return the period of the controller.
   */
  public double getPeriod() {
    return m_period;
  }

  /**
   * Returns the position tolerance of this controller.
   *
   * @return the position tolerance of the controller.
   */
  public double getPositionTolerance() {
    return m_positionTolerance;
  }

  /**
   * Returns the velocity tolerance of this controller.
   *
   * @return the velocity tolerance of the controller.
   */
  public double getVelocityTolerance() {
    return m_velocityTolerance;
  }

  /**
   * Normalizes angle to (-180, 180) from any angle.
   *
   * @param angle angle to normalize.
   * @return normalized angle.
   */
  private double normalize(double angle) {
    while (angle < -180) {
      angle += 360;
    }
    while (angle > 180) {
      angle -= 360;
    }
    return angle;
  }

  /**
   * Changes a (-180, 180) angle to a (0, 360) angle
   *
   * @param angle angle in deg in the coordinate system (-180, 180).
   * @return changed angle.
   */
  private double to_360(double angle) {
    if (angle > 90) {
      return 450 - angle;
    } else {
      return 90 - angle;
    }
  }

  /**
   * Is an angle within the range of angles a and b?
   *
   * @param test_angle angle to test.
   * @param a first angle.
   * @param b second angle.
   * @return If the test angle is between the first and second angle.
   */
  private boolean is_within_range(double test_angle, double a, double b) {
    test_angle = to_360(test_angle);
    // System.out.println(test_angle);
    a = to_360(a);
    // System.out.println(a);
    b = to_360(b);
    // System.out.println(b);
    a -= test_angle;
    b -= test_angle;
    normalize(a);
    normalize(b);
    if (a * b > 0) {
      return false;
    }
    return Math.abs(a - b) < 180;
  }

  /**
   * Set an avoidance range where you do not want the controlled mechanism to go. When setpoint is
   * within the range, changes the setpoint to be max/min +/- tolerance. When setpoint is calculated
   * to pass through the avoidance range, inverts the angle so that it goes the opposite way to meet
   * setpoint. Assumes that the areas between min/max is less than 180 deg, will invert the angle if
   * over. Assumes a default tolerance of 5 deg.
   *
   * @param minimum minimum of the range. Assumes CCW+, so minimum is with that regard.
   * @param maximum maximum of the range. Assumes CCW+, so maximum is with that regard.
   */
  public void setAvoidanceRange(Rotation2d minimum, Rotation2d maximum) {
    m_minimumAvoidanceBound = minimum.rotateBy(new Rotation2d());
    m_maximumAvoidanceBound = maximum.rotateBy(new Rotation2d());
    m_middleAvoidanceBound =
        m_minimumAvoidanceBound.plus(m_maximumAvoidanceBound.minus(m_minimumAvoidanceBound).div(2));
  }

  /**
   * Set an avoidance range where you do not want the controlled mechanism to go. When setpoint is
   * within the range, changes the setpoint to be max/min +/- tolerance. When setpoint is calculated
   * to pass through the avoidance range, inverts the angle so that it goes the opposite way to meet
   * setpoint. Assumes that the areas between min/max is less than 180 deg, will invert the angle if
   * over.
   *
   * @param minimum minimum of the range. Assumes CCW+, so minimum is with that regard.
   * @param maximum maximum of the range. Assumes CCW+, so maximum is with that regard.
   * @param tolerance tolerance of the range.
   */
  public void setAvoidanceRange(Rotation2d minimum, Rotation2d maximum, Rotation2d tolerance) {
    if (maximum.minus(minimum).getDegrees() < 0) {
      throw new IllegalArgumentException(
          "Ensure that maximum is greater than minimum and the angle between the two does now exceed 180 deg.");
    }
    m_minimumAvoidanceBound = minimum.minus(Rotation2d.fromDegrees(0));
    m_maximumAvoidanceBound = maximum.minus(Rotation2d.fromDegrees(0));
    m_avoidanceTolerance = tolerance.minus(Rotation2d.fromDegrees(0));
    m_middleAvoidanceBound =
        m_minimumAvoidanceBound.plus(m_maximumAvoidanceBound.minus(m_minimumAvoidanceBound).div(2));
  }

  /**
   * Is the setpoint within the defined avoidance range?
   *
   * @return if the setpoint is within the avoidange range
   */
  private boolean isSetpointWithinObstacle() {
    return is_within_range(
        m_setpoint.getDegrees(),
        m_maximumAvoidanceBound.getDegrees(),
        m_minimumAvoidanceBound.getDegrees());
  }

  /**
   * Is the defined avoidance range in the way of the setpoint?
   *
   * @return if avoidance range is between the setpoint and the measurement
   */
  private boolean isObstacleInSetpointsWay() {
    return
    /** Are both the maximum and minimum in between the setpoint and measurement? */
    (is_within_range(
                m_maximumAvoidanceBound.getDegrees(),
                m_setpoint.getDegrees(),
                m_measurement.getDegrees())
            && is_within_range(
                m_minimumAvoidanceBound.getDegrees(),
                m_setpoint.getDegrees(),
                m_measurement.getDegrees()))
        /**
         * Case for when measurement is within the obstacle but closer to the maximum and setpoint
         * is on opposite side (could still be within obstacle)
         */
        || (is_within_range(
                m_measurement.getDegrees(),
                m_middleAvoidanceBound.getDegrees(),
                m_maximumAvoidanceBound.getDegrees())
            && is_within_range(
                m_minimumAvoidanceBound.getDegrees(),
                m_setpoint.getDegrees(),
                m_measurement.getDegrees()))
        /**
         * Case for when measurement is within the obstacle but closer to the minimum and setpoint
         * is on opposite side (could still be within obstacle)
         */
        || (is_within_range(
                m_measurement.getDegrees(),
                m_minimumAvoidanceBound.getDegrees(),
                m_middleAvoidanceBound.getDegrees())
            && is_within_range(
                m_maximumAvoidanceBound.getDegrees(),
                m_setpoint.getDegrees(),
                m_measurement.getDegrees()));
  }

  /**
   * Sets the setpoint for the PIDController.
   *
   * @param setpoint The desired setpoint.
   */
  public void setSetpoint(Rotation2d setpoint) {
    m_setpoint = setpoint.rotateBy(new Rotation2d());
    m_haveSetpoint = true;
  }

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  public double getSetpoint() {
    return m_setpoint.getRadians();
  }

  /**
   * Returns true if the error is within the tolerance of the setpoint.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @return Whether the error is within the acceptable bounds.
   */
  public boolean atSetpoint() {
    return m_haveMeasurement
        && m_haveSetpoint
        && Math.abs(m_positionError.getRadians()) < m_positionTolerance
        && Math.abs(m_velocityError) < m_velocityTolerance;
  }

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * <p>When the cap is reached, the integrator value is added to the controller output rather than
   * the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    m_minimumIntegral = minimumIntegral;
    m_maximumIntegral = maximumIntegral;
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   */
  public void setTolerance(double positionTolerance) {
    setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_positionTolerance = positionTolerance;
    m_velocityTolerance = velocityTolerance;
  }

  /**
   * Returns the difference between the setpoint and the measurement.
   *
   * @return The error.
   */
  public double getPositionError() {
    return m_positionError.getRadians();
  }

  /**
   * Returns the velocity error.
   *
   * @return The velocity error.
   */
  public double getVelocityError() {
    return m_velocityError;
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param setpoint The new setpoint of the controller.
   * @return The next controller output.
   */
  public double calculate(Rotation2d measurement, Rotation2d setpoint) {
    m_setpoint = setpoint.rotateBy(new Rotation2d());
    m_haveSetpoint = true;
    return calculate(measurement);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @return The next controller output.
   */
  public double calculate(Rotation2d measurement) {
    m_measurement = measurement.rotateBy(new Rotation2d());
    m_prevError = m_positionError;
    m_haveMeasurement = true;

    // System.out.println("initSetp: " + m_setpoint.getRadians());

    if (isSetpointWithinObstacle()) {
      /* Check which bound is closer, so we can use that plus/minus a tolerance for our setpoint. */
      // System.out.println("Setp in obstacle: " + m_setpoint.getRadians());
      if (m_setpoint.minus(m_minimumAvoidanceBound).getDegrees()
          > m_maximumAvoidanceBound.minus(m_setpoint).getDegrees()) {
        m_setpoint = m_maximumAvoidanceBound.plus(m_avoidanceTolerance);
      } else {
        m_setpoint = m_minimumAvoidanceBound.minus(m_avoidanceTolerance);
      }
    }
    // System.out.println("finalSetp: " + m_setpoint.getRadians());

    Rotation2d initialError = m_setpoint.minus(m_measurement);

    if (isObstacleInSetpointsWay()) {
      m_positionError =
          initialError.getDegrees() >= 0
              ? Rotation2d.fromDegrees(-360 + initialError.getDegrees())
              : Rotation2d.fromDegrees(360 + initialError.getDegrees());
      // System.out.println("obstacle in way: " + m_positionError.getRadians());
    } else {
      m_positionError = initialError;
      // System.out.println("obstacle not in way: " + m_positionError.getRadians());
    }

    m_velocityError = (m_positionError.getRadians() - m_prevError.getRadians()) / m_period;

    if (m_ki != 0) {
      m_totalError =
          MathUtil.clamp(
              m_totalError + m_positionError.getRadians() * m_period,
              m_minimumIntegral / m_ki,
              m_maximumIntegral / m_ki);
    }

    return m_kp * m_positionError.getRadians() + m_ki * m_totalError + m_kd * m_velocityError;
  }

  /** Resets the previous error and the integral term. */
  public void reset() {
    m_positionError = new Rotation2d();
    m_prevError = new Rotation2d();
    m_totalError = 0;
    m_velocityError = 0;
    m_haveMeasurement = false;
  }
}

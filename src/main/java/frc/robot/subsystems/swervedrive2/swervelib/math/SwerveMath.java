package frc.robot.subsystems.swervedrive2.swervelib.math;

public class SwerveMath
{

  /**
   * Calculate the angle kV which will be multiplied by the radians per second for the feedforward. Volt * seconds /
   * degree <=> (maxVolts) / (maxSpeed)
   *
   * @param optimalVoltage    Optimal voltage to use when calculating the angle kV.
   * @param motorFreeSpeedRPM Motor free speed in Rotations per Minute.
   * @param angleGearRatio    Angle gear ratio, the amount of times the motor as to turn for the wheel rotation.
   * @return angle kV for feedforward.
   */
  public static double calculateAngleKV(double optimalVoltage, double motorFreeSpeedRPM, double angleGearRatio)
  {
    double maxAngularVelocity = 360 * (motorFreeSpeedRPM / angleGearRatio) / 60; // deg/s
    return optimalVoltage / maxAngularVelocity;
  }

  /**
   * Calculate the meters per rotation for the integrated encoder. Calculation: 4in diameter wheels * pi [circumfrence]
   * / gear ratio.
   *
   * @param wheelDiameter  Wheel diameter in meters.
   * @param driveGearRatio The gear ratio of the drive motor.
   * @return Meters per rotation for the drive motor.
   */
  public static double calculateMetersPerRotation(double wheelDiameter, double driveGearRatio)
  {
    return (Math.PI * wheelDiameter) / driveGearRatio;
  }

  /**
   * Calculate the degrees per steering rotation for the integrated encoder. Encoder conversion values.  Drive converts
   * motor rotations to linear wheel distance and steering converts motor rotations to module azimuth.
   *
   * @param angleGearRatio The gear ratio of the steering motor.
   * @return Degrees per steering rotation for the angle motor.
   */
  public static double calculateDegreesPerSteeringRotation(double angleGearRatio)
  {
    return 360 / angleGearRatio;
  }

  /**
   * Calculate the maximum angular velocity.
   *
   * @param maxSpeed        Max speed of the robot in meters per second.
   * @param furthestModuleX X of the furthest module in meters.
   * @param furthestModuleY Y of the furthest module in meters.
   * @return Maximum angular velocity in rad/s.
   */
  public static double calculateMaxAngularVelocity(double maxSpeed, double furthestModuleX, double furthestModuleY)
  {
    return maxSpeed / Math.hypot(furthestModuleX, furthestModuleY);
  }

  /**
   * Calculate the practical maximum acceleration of the robot using the wheel coefficient of friction.
   *
   * @param cof Coefficient of Friction of the wheel grip tape.
   * @return Practical maximum acceleration in m/s/s.
   */
  public static double calculateMaxAcceleration(double cof)
  {
    return cof * 9.81;
  }

  /**
   * Calculate the maximum theoretical acceleration without friction.
   *
   * @param stallTorqueNm Stall torque of driving motor in nM.
   * @param gearRatio     Gear ratio for driving motor number of motor rotations until one wheel rotation.
   * @param moduleCount   Number of swerve modules.
   * @param wheelDiameter Wheel diameter in meters.
   * @param robotMass     Mass of the robot in kg.
   * @return Theoretical maximum acceleration in m/s/s.
   */
  public static double calculateMaxAcceleration(double stallTorqueNm, double gearRatio, double moduleCount,
                                                double wheelDiameter, double robotMass)
  {
    return (stallTorqueNm * gearRatio * moduleCount) / ((wheelDiameter / 2) * robotMass);
  }

}

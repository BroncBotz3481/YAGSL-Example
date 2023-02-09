package frc.robot.subsystems.swervedrive2.swervelib.parser;

public class SwerveModulePhysicalCharacteristics
{

  /**
   * Wheel diameter in meters.
   */
  public final double wheelDiameter;
  /**
   * Drive gear ratio. How many times the motor has to spin before the wheel makes a complete rotation.
   */
  public final double driveGearRatio;
  /**
   * Angle gear ratio. How many times the motor has to spin before the wheel makes a full rotation.
   */
  public final double angleGearRatio;
  /**
   * Optimal voltage of the robot.
   */
  public final double optimalVoltage;
  /**
   * Wheel grip tape coefficient of friction on carpet, as described by the vendor.
   */
  public final double wheelGripCoefficientOfFriction;
  /**
   * Free speed rotations per minute of the motor, as described by the vendor.
   */
  public final double motorFreeSpeedRPM;

  /**
   * Construct the swerve module physical characteristics.
   *
   * @param driveGearRatio                 Gear ratio of the drive motor. Number of motor rotations to rotate the
   *                                       wheel.
   * @param angleGearRatio                 Gear ratio of the angle motor. Number of motor rotations to spin the wheel.
   * @param motorFreeSpeedRPM              Motor free speed rotation per minute.
   * @param wheelDiameter                  Wheel diameter in meters.
   * @param wheelGripCoefficientOfFriction Wheel grip coefficient of friction on carpet given by manufacturer.
   * @param optimalVoltage                 Optimal robot voltage.
   */
  public SwerveModulePhysicalCharacteristics(double driveGearRatio, double angleGearRatio, double motorFreeSpeedRPM,
                                             double wheelDiameter, double wheelGripCoefficientOfFriction,
                                             double optimalVoltage)
  {
    this.wheelGripCoefficientOfFriction = wheelGripCoefficientOfFriction;
    this.optimalVoltage = optimalVoltage;

    this.motorFreeSpeedRPM = motorFreeSpeedRPM;
    this.angleGearRatio = angleGearRatio;
    this.driveGearRatio = driveGearRatio;
    this.wheelDiameter = wheelDiameter;
  }

  /**
   * Construct the swerve module physical characteristics. Assume coefficient of friction is 1.19 and optimal voltage is
   * 12v.
   *
   * @param driveGearRatio    Gear ratio of the drive motor. Number of motor rotations to rotate the wheel.
   * @param angleGearRatio    Gear ratio of the angle motor. Number of motor rotations to spin the wheel.
   * @param motorFreeSpeedRPM Motor free speed rotation per minute.
   * @param wheelDiameter     Wheel diameter in meters.
   */
  public SwerveModulePhysicalCharacteristics(double driveGearRatio, double angleGearRatio, double motorFreeSpeedRPM,
                                             double wheelDiameter)
  {
    this(driveGearRatio, angleGearRatio, motorFreeSpeedRPM, wheelDiameter, 1.19, 12);
  }
}

package frc.robot.subsystems.swervedrive2.swervelib.parser;

import edu.wpi.first.math.util.Units;

public class SwerveModulePhysicalCharacteristics
{

  /**
   * Wheel diameter in meters.
   */
  public final double wheelDiameter                  = Units.inchesToMeters(4);
  /**
   * Drive gear ratio. How many times the motor has to spin before the wheel makes a complete rotation.
   */
  public final double driveGearRatio                 = 6.75;
  /**
   * Angle gear ratio. How many times the motor has to spin before the wheel makes a full rotation.
   */
  public final double angleGearRatio                 = 12.8;
  /**
   * Optimal voltage of the robot.
   */
  public final double optimalVoltage                 = 12;
  /**
   * Wheel grip tape coefficient of friction on carpet, as described by the vendor.
   */
  public final double wheelGripCoefficientOfFriction = 1.19;
  /**
   * Free speed rotations per minute of the motor, as described by the vendor.
   */
  public final double motorFreeSpeedRPM              = 5676;
}

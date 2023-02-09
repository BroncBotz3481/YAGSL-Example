package frc.robot.subsystems.swervedrive2.parser;

import static frc.robot.subsystems.swervedrive2.math.SwerveMath.calculateAngleKV;
import static frc.robot.subsystems.swervedrive2.math.SwerveMath.calculateDegreesPerSteeringRotation;
import static frc.robot.subsystems.swervedrive2.math.SwerveMath.calculateMaxAcceleration;
import static frc.robot.subsystems.swervedrive2.math.SwerveMath.calculateMetersPerRotation;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swervedrive2.encoders.CANCoderSwerve;
import frc.robot.subsystems.swervedrive2.encoders.SwerveAbsoluteEncoder;
import frc.robot.subsystems.swervedrive2.motors.SparkMaxSwerve;
import frc.robot.subsystems.swervedrive2.motors.SwerveMotor;

public class SwerveModuleConfiguration
{

  public final int    driveMotorID;
  public final int    angleMotorID;
  public final int    absoluteEncoderID;
  public final String driveMotorCANBus, angleMotorCANBus, absoluteEncoderCANBus;
  public final double        angleOffset;
  public final boolean       absoluteEncoderInverted;
  public final boolean       driveMotorInverted;
  public final double        wheelDiameter                  = Units.inchesToMeters(4);
  public final double        driveGearRatio                 = 6.75;
  public final double        angleGearRatio                 = 12.8;
  public final double        maxSpeed;
  public final double        optimalVoltage                 = 12;
  public final double        wheelGripCoefficientOfFriction = 1.19;
  public final double        motorFreeSpeedRPM              = 5676;
  public       PIDFConfig    anglePIDF;
  public       PIDFConfig    velocityPIDF;
  public       double        angleKV                        = calculateAngleKV(optimalVoltage,
                                                                               motorFreeSpeedRPM,
                                                                               angleGearRatio);
  public       Translation2d moduleLocation;


  /**
   * Construct a configuration object for swerve modules.
   *
   * @param driveMotorID            Drive motor CAN ID or pin ID.
   * @param angleMotorID            Angle motor CAN ID or pin ID.
   * @param absoluteEncoderID       Absolute encoder CAN ID or pin ID.
   * @param angleOffset             Absolute angle offset to 0.
   * @param absoluteEncoderInverted Absolute encoder inverted.
   * @param driveMotorInverted      Drive motor inverted.
   * @param absoluteEncoderCANBus   CAN Bus name where absolute encoder is attached.
   * @param angleMotorCANBus        CAN Bus name where the angle motor is attached.
   * @param driveMotorCANBus        CAN Bus name where the drive motor is attached.
   * @param xMeters                 Module location in meters from the center horizontally.
   * @param yMeters                 Module location in meters from center vertically.
   * @param anglePIDF               Angle PIDF configuration.
   * @param velocityPIDF            Velocity PIDF configuration.
   * @param maxSpeed                Maximum speed in meters per second.
   */
  public SwerveModuleConfiguration(int driveMotorID, int angleMotorID, int absoluteEncoderID, double angleOffset,
                                   boolean absoluteEncoderInverted, boolean driveMotorInverted, String driveMotorCANBus,
                                   String angleMotorCANBus, String absoluteEncoderCANBus, double xMeters,
                                   double yMeters, PIDFConfig anglePIDF, PIDFConfig velocityPIDF, double maxSpeed)
  {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.absoluteEncoderID = absoluteEncoderID;
    this.angleOffset = angleOffset;
    this.absoluteEncoderInverted = absoluteEncoderInverted;
    this.driveMotorInverted = driveMotorInverted;
    this.driveMotorCANBus = driveMotorCANBus;
    this.angleMotorCANBus = angleMotorCANBus;
    this.absoluteEncoderCANBus = absoluteEncoderCANBus;
    this.moduleLocation = new Translation2d(xMeters, yMeters);
    this.anglePIDF = anglePIDF;
    this.velocityPIDF = velocityPIDF;
    this.maxSpeed = maxSpeed;
  }

  /**
   * Construct a configuration object for swerve modules.
   *
   * @param driveMotorID      Drive motor CAN ID or pin ID.
   * @param angleMotorID      Angle motor CAN ID or pin ID.
   * @param absoluteEncoderID Absolute encoder CAN ID or pin ID.
   * @param angleOffset       Absolute angle offset to 0.
   * @param xMeters           Module location in meters from the center horizontally.
   * @param yMeters           Module location in meters from center vertically.
   * @param anglePIDF         Angle PIDF configuration.
   * @param velocityPIDF      Velocity PIDF configuration.
   * @param maxSpeed          Maximum robot speed in meters per second.
   */
  public SwerveModuleConfiguration(int driveMotorID, int angleMotorID, int absoluteEncoderID, double angleOffset,
                                   double xMeters, double yMeters, PIDFConfig anglePIDF, PIDFConfig velocityPIDF,
                                   double maxSpeed)
  {
    this(driveMotorID,
         angleMotorID,
         absoluteEncoderID,
         angleOffset,
         false,
         false,
         null,
         null,
         null,
         xMeters,
         yMeters,
         anglePIDF,
         velocityPIDF,
         maxSpeed);
  }


  /**
   * Create the drive feedforward for swerve modules.
   *
   * @return Drive feedforward for drive motor on a swerve module.
   */
  public SimpleMotorFeedforward createDriveFeedforward()
  {
    double kv = optimalVoltage / maxSpeed;
    double ka = optimalVoltage / calculateMaxAcceleration(wheelGripCoefficientOfFriction);
    return new SimpleMotorFeedforward(0, kv, ka);
  }

  /**
   * Create the {@link SwerveMotor} for the drive motor.
   *
   * @return Instantiated SwerveMotor.
   */
  public SwerveMotor createDriveMotor()
  {
    return new SparkMaxSwerve(driveMotorID, true);
  }

  /**
   * Create the {@link SwerveMotor} for the drive motor.
   *
   * @return Instantiated SwerveMotor.
   */
  public SwerveMotor createAngleMotor()
  {
    return new SparkMaxSwerve(angleMotorID, false);
  }

  /**
   * Create the {@link SwerveAbsoluteEncoder} for the given configuration.
   *
   * @return Instantiated Absolute encoder.
   */
  public SwerveAbsoluteEncoder createAbsoluteEncoder()
  {
    return new CANCoderSwerve(absoluteEncoderID);
  }


  /**
   * Get the encoder conversion for position encoders.
   *
   * @param isDriveMotor For the drive motor.
   * @return Position encoder conversion factor.
   */
  public double getPositionEncoderConversion(boolean isDriveMotor)
  {
    return isDriveMotor ? calculateMetersPerRotation(wheelDiameter, driveGearRatio)
                        : calculateDegreesPerSteeringRotation(angleGearRatio);
  }
}

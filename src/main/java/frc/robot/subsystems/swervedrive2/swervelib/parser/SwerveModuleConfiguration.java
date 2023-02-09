package frc.robot.subsystems.swervedrive2.swervelib.parser;

import static frc.robot.subsystems.swervedrive2.swervelib.math.SwerveMath.calculateAngleKV;
import static frc.robot.subsystems.swervedrive2.swervelib.math.SwerveMath.calculateDegreesPerSteeringRotation;
import static frc.robot.subsystems.swervedrive2.swervelib.math.SwerveMath.calculateMaxAcceleration;
import static frc.robot.subsystems.swervedrive2.swervelib.math.SwerveMath.calculateMetersPerRotation;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swervedrive2.swervelib.encoders.CANCoderSwerve;
import frc.robot.subsystems.swervedrive2.swervelib.encoders.SwerveAbsoluteEncoder;
import frc.robot.subsystems.swervedrive2.swervelib.motors.SparkMaxSwerve;
import frc.robot.subsystems.swervedrive2.swervelib.motors.SwerveMotor;

public class SwerveModuleConfiguration
{

  /**
   * Drive motor CAN ID or pin ID.
   */
  public final int    driveMotorID;
  /**
   * Angle motor CAN ID or pin ID.
   */
  public final int    angleMotorID;
  /**
   * Absolute encoder CAN ID or pin ID.
   */
  public final int    absoluteEncoderID;
  /**
   * CAN bus name for the drive/angle motor, or absolute encoder.
   */
  public final String driveMotorCANBus, angleMotorCANBus, absoluteEncoderCANBus;
  /**
   * Angle offset in degrees for the Swerve Module.
   */
  public final double                              angleOffset;
  /**
   * Whether the absolute encoder is inverted.
   */
  public final boolean                             absoluteEncoderInverted;
  /**
   * Whether the drive motor is inverted.
   */
  public final boolean                             driveMotorInverted;
  /**
   * Maximum robot speed in meters per second.
   */
  public final double                              maxSpeed;
  /**
   * PIDF configuration options for the angle motor closed-loop PID controller.
   */
  public       PIDFConfig                          anglePIDF;
  /**
   * PIDF configuration options for the drive motor closed-loop PID controller.
   */
  public       PIDFConfig                          velocityPIDF;
  /**
   * Angle volt-meter-per-second.
   */
  public       double                              angleKV;
  /**
   * Swerve module location relative to the robot.
   */
  public       Translation2d                       moduleLocation;
  /**
   * Physical characteristics of the swerve module.
   */
  public       SwerveModulePhysicalCharacteristics physicalCharacteristics;


  /**
   * Construct a configuration object for swerve modules.
   *
   * @param driveMotorID            Drive motor CAN ID or pin ID.
   * @param angleMotorID            Angle motor CAN ID or pin ID.
   * @param absoluteEncoderID       Absolute encoder CAN ID or pin ID.
   * @param angleOffset             Absolute angle offset to 0.
   * @param absoluteEncoderInverted Absolute encoder inverted.
   * @param driveMotorInverted      Drive motor inverted.
   * @param absoluteEncoderCANBus   CAN Bus name where absolute encoder is attached. Can be null if none.
   * @param angleMotorCANBus        CAN Bus name where the angle motor is attached. Can be null if none.
   * @param driveMotorCANBus        CAN Bus name where the drive motor is attached. Can be null if none.
   * @param xMeters                 Module location in meters from the center horizontally.
   * @param yMeters                 Module location in meters from center vertically.
   * @param anglePIDF               Angle PIDF configuration.
   * @param velocityPIDF            Velocity PIDF configuration.
   * @param maxSpeed                Maximum speed in meters per second.
   * @param physicalCharacteristics Physical characteristics of the swerve module.
   */
  public SwerveModuleConfiguration(int driveMotorID, int angleMotorID, int absoluteEncoderID, double angleOffset,
                                   boolean absoluteEncoderInverted, boolean driveMotorInverted, String driveMotorCANBus,
                                   String angleMotorCANBus, String absoluteEncoderCANBus, double xMeters,
                                   double yMeters, PIDFConfig anglePIDF, PIDFConfig velocityPIDF, double maxSpeed,
                                   SwerveModulePhysicalCharacteristics physicalCharacteristics)
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
    this.angleKV = calculateAngleKV(physicalCharacteristics.optimalVoltage,
                                    physicalCharacteristics.motorFreeSpeedRPM,
                                    physicalCharacteristics.angleGearRatio);
    this.physicalCharacteristics = physicalCharacteristics;

  }

  /**
   * Construct a configuration object for swerve modules.
   *
   * @param driveMotorID            Drive motor CAN ID or pin ID.
   * @param angleMotorID            Angle motor CAN ID or pin ID.
   * @param absoluteEncoderID       Absolute encoder CAN ID or pin ID.
   * @param angleOffset             Absolute angle offset to 0.
   * @param xMeters                 Module location in meters from the center horizontally.
   * @param yMeters                 Module location in meters from center vertically.
   * @param anglePIDF               Angle PIDF configuration.
   * @param velocityPIDF            Velocity PIDF configuration.
   * @param maxSpeed                Maximum robot speed in meters per second.
   * @param physicalCharacteristics Physical characteristics of the swerve module.
   */
  public SwerveModuleConfiguration(int driveMotorID, int angleMotorID, int absoluteEncoderID, double angleOffset,
                                   double xMeters, double yMeters, PIDFConfig anglePIDF, PIDFConfig velocityPIDF,
                                   double maxSpeed, SwerveModulePhysicalCharacteristics physicalCharacteristics)
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
         maxSpeed,
         physicalCharacteristics);
  }


  /**
   * Create the drive feedforward for swerve modules.
   *
   * @return Drive feedforward for drive motor on a swerve module.
   */
  public SimpleMotorFeedforward createDriveFeedforward()
  {
    double kv = physicalCharacteristics.optimalVoltage / maxSpeed;
    ///^ Volt-seconds per meter (max voltage divided by max speed)
    double ka = physicalCharacteristics.optimalVoltage /
                calculateMaxAcceleration(physicalCharacteristics.wheelGripCoefficientOfFriction);
    ///^ Volt-seconds^2 per meter (max voltage divided by max accel)
    return new SimpleMotorFeedforward(0, kv, ka);
  }

  /**
   * Create the {@link SwerveMotor} for the drive motor.
   *
   * @return Instantiated SwerveMotor.
   */
  public SwerveMotor createDriveMotor()
  {
    SwerveMotor motor = new SparkMaxSwerve(driveMotorID, true);
    return motor;
  }

  /**
   * Create the {@link SwerveMotor} for the drive motor.
   *
   * @return Instantiated SwerveMotor.
   */
  public SwerveMotor createAngleMotor()
  {
    SwerveMotor motor = new SparkMaxSwerve(angleMotorID, false);
    return motor;
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
    return isDriveMotor ? calculateMetersPerRotation(physicalCharacteristics.wheelDiameter,
                                                     physicalCharacteristics.driveGearRatio)
                        : calculateDegreesPerSteeringRotation(physicalCharacteristics.angleGearRatio);
  }
}

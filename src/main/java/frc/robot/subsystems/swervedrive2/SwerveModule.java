package frc.robot.subsystems.swervedrive2;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive2.encoders.CANCoderSwerve;
import frc.robot.subsystems.swervedrive2.encoders.SwerveAbsoluteEncoder;
import frc.robot.subsystems.swervedrive2.math.BetterSwerveModuleState;
import frc.robot.subsystems.swervedrive2.motors.SparkMaxSwerve;
import frc.robot.subsystems.swervedrive2.motors.SwerveMotor;
import frc.robot.subsystems.swervedrive2.parser.SwerveDriveConfiguration;
import frc.robot.subsystems.swervedrive2.parser.SwerveModuleConfiguration;

public class SwerveModule
{

  /**
   * Angle offset from the absolute encoder.
   */
  private final double      angleOffset;
  /**
   * Swerve Motors.
   */
  private final SwerveMotor angleMotor, driveMotor;
  /**
   * Absolute encoder for swerve drive.
   */
  private final SwerveAbsoluteEncoder absoluteEncoder;
  /**
   * Module number for kinematics, usually 0 to 3. front left -> front right -> back left -> back right.
   */
  public        int                   moduleNumber;
  /**
   * Feedforward for drive motor during closed loop control.
   */
  SimpleMotorFeedforward feedforward = SwerveDriveConfiguration.createDriveFeedforward();
  /**
   * Last angle set for the swerve module.
   */
  private double lastAngle;
  /**
   * Current state.
   */
  private double angle, omega, speed, fakePos, lastTime, dt;
  /**
   * Swerve module configuration options.
   */
  private final SwerveModuleConfiguration configuration;
  /**
   * Timer for simulation.
   */
  private       Timer                     time;

  /**
   * Construct the swerve module and initialize the swerve module motors and absolute encoder.
   *
   * @param moduleNumber    Module number for kinematics.
   * @param moduleConstants Module constants containing CAN ID's and offsets.
   */
  public SwerveModule(int moduleNumber, SwerveModuleConfiguration moduleConstants)
  {
    angle = 0;
    speed = 0;
    omega = 0;
    fakePos = 0;
    this.moduleNumber = moduleNumber;
    configuration = moduleConstants;
    angleOffset = moduleConstants.angleOffset;

    angleMotor = new SparkMaxSwerve(moduleConstants.angleMotorID);
    driveMotor = new SparkMaxSwerve(moduleConstants.driveMotorID);
    angleMotor.factoryDefaults();
    driveMotor.factoryDefaults();

    // Config angle encoders
    absoluteEncoder = new CANCoderSwerve(moduleConstants.cancoderID);
    absoluteEncoder.factoryDefault();
    absoluteEncoder.configure(moduleConstants.absoluteEncoderInverted);

    angleMotor.configureIntegratedEncoder(false);
    angleMotor.setPosition(absoluteEncoder.getAbsolutePosition() - angleOffset);

    // Config angle motor/controller
    angleMotor.configurePIDF(false,
                             moduleConstants.angleKP,
                             moduleConstants.angleKI,
                             moduleConstants.angleKD,
                             moduleConstants.angleKF,
                             moduleConstants.angleKIZ);
    angleMotor.configurePIDWrapping(-180, 180);
    angleMotor.setMotorBrake(false);

    // Config drive motor/controller
    driveMotor.configureIntegratedEncoder(true);
    driveMotor.configurePIDF(true,
                             moduleConstants.velocityKP,
                             moduleConstants.velocityKI,
                             moduleConstants.velocityKD,
                             moduleConstants.velocityKF,
                             moduleConstants.velocityKIZ);
    driveMotor.setInverted(moduleConstants.driveMotorInverted);
    driveMotor.setMotorBrake(true);

    driveMotor.burnFlash();
    angleMotor.burnFlash();

    lastAngle = getState().angle.getDegrees();

    if (!Robot.isReal())
    {
      time = new Timer();
      time.start();
      lastTime = time.get();
    }
  }

  /**
   * Set the desired state of the swerve module.
   *
   * @param desiredState Desired swerve module state.
   * @param isOpenLoop   Whether to use open loop (direct percent) or direct velocity control.
   */
  public void setDesiredState(BetterSwerveModuleState desiredState, boolean isOpenLoop)
  {
    SwerveModuleState simpleState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    simpleState = SwerveModuleState.optimize(simpleState, getState().angle);
    desiredState = new BetterSwerveModuleState(simpleState.speedMetersPerSecond, simpleState.angle,
                                               desiredState.omegaRadPerSecond);

    SmartDashboard.putNumber("Optimized " + moduleNumber + " Speed Setpoint: ", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("Optimized " + moduleNumber + " Angle Setpoint: ", desiredState.angle.getDegrees());
    SmartDashboard.putNumber("Module " + moduleNumber + " Omega: ", Math.toDegrees(desiredState.omegaRadPerSecond));

    if (isOpenLoop)
    {
      double percentOutput = desiredState.speedMetersPerSecond / configuration.maxSpeed;
      driveMotor.set(percentOutput);
    } else
    {
      double velocity = desiredState.speedMetersPerSecond;
      driveMotor.setReference(true, velocity, feedforward.calculate(velocity));
    }

    // Prevents module rotation if speed is less than 1%
    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (configuration.maxSpeed * 0.01) ?
                    lastAngle :
                    desiredState.angle.getDegrees());
    angleMotor.setReference(false, angle, Math.toDegrees(desiredState.omegaRadPerSecond) * configuration.angleKZ);
    lastAngle = angle;

    if (!Robot.isReal())
    {
      dt = time.get() - lastTime;
      fakePos += (speed * dt);
      lastTime = time.get();
    }

    this.angle = desiredState.angle.getDegrees();
    omega = desiredState.omegaRadPerSecond;
    speed = desiredState.speedMetersPerSecond;
  }

  /**
   * Get the Swerve Module state.
   *
   * @return Current SwerveModule state.
   */
  public BetterSwerveModuleState getState()
  {
    double     velocity;
    Rotation2d azimuth;
    double     omega;
    if (Robot.isReal())
    {
      velocity = driveMotor.getVelocity();
      azimuth = Rotation2d.fromDegrees(angleMotor.getPosition());
      omega = angleMotor.getVelocity();
    } else
    {
      velocity = speed;
      azimuth = Rotation2d.fromDegrees(this.angle);
      omega = this.omega;
    }
    return new BetterSwerveModuleState(velocity, azimuth, omega);
  }

  public SwerveModulePosition getPosition()
  {
    double     position;
    Rotation2d azimuth;
    if (Robot.isReal())
    {
      position = driveMotor.getPosition();
      azimuth = Rotation2d.fromDegrees(angleMotor.getPosition());
    } else
    {
      position = fakePos;
      azimuth = Rotation2d.fromDegrees(angle + (Math.toDegrees(omega) * dt));
    }
    SmartDashboard.putNumber("Module " + moduleNumber + "Angle", azimuth.getDegrees());
    return new SwerveModulePosition(position, azimuth);
  }

  /**
   * Get the CANCoder absolute position.
   *
   * @return Absolute encoder angle in degrees.
   */
  public double getCANCoder()
  {
    return absoluteEncoder.getAbsolutePosition();
  }

  /**
   * Get the relative encoder angle in degrees.
   *
   * @return Angle in degrees.
   */
  public double getRelativeEncoder()
  {
    return angleMotor.getPosition();
  }

  /**
   * Set the brake mode.
   *
   * @param brake Set the brake mode.
   */
  public void setMotorBrake(boolean brake)
  {
    driveMotor.setMotorBrake(brake);
  }
}

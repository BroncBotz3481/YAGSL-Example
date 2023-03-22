package swervelib;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveModuleState2;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.simulation.SwerveModuleSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The Swerve Module class which represents and controls Swerve Modules for the swerve drive.
 */
public class SwerveModule
{

  /**
   * Swerve module configuration options.
   */
  public final  SwerveModuleConfiguration configuration;
  /**
   * Angle offset from the absolute encoder.
   */
  private final double                    angleOffset;
  /**
   * Swerve Motors.
   */
  private final SwerveMotor               angleMotor, driveMotor;
  /**
   * Absolute encoder for swerve drive.
   */
  private final SwerveAbsoluteEncoder  absoluteEncoder;
  /**
   * Module number for kinematics, usually 0 to 3. front left -> front right -> back left -> back right.
   */
  public        int                    moduleNumber;
  /**
   * Feedforward for drive motor during closed loop control.
   */
  public        SimpleMotorFeedforward feedforward;
  /**
   * Last angle set for the swerve module.
   */
  public        double                 lastAngle;
  /**
   * Last velocity set for the swerve module.
   */
  public        double                 lastVelocity;
  /**
   * Simulated swerve module.
   */
  private       SwerveModuleSimulation simModule;
  /**
   * Encoder synchronization queued.
   */
  private       boolean                synchronizeEncoderQueued = false;

  /**
   * Construct the swerve module and initialize the swerve module motors and absolute encoder.
   *
   * @param moduleNumber        Module number for kinematics.
   * @param moduleConfiguration Module constants containing CAN ID's and offsets.
   */
  public SwerveModule(int moduleNumber, SwerveModuleConfiguration moduleConfiguration)
  {
    //    angle = 0;
    //    speed = 0;
    //    omega = 0;
    //    fakePos = 0;
    this.moduleNumber = moduleNumber;
    configuration = moduleConfiguration;
    angleOffset = moduleConfiguration.angleOffset;

    // Initialize Feedforward for drive motor.
    feedforward = configuration.createDriveFeedforward();

    // Create motors from configuration and reset them to defaults.
    angleMotor = moduleConfiguration.angleMotor;
    driveMotor = moduleConfiguration.driveMotor;
    angleMotor.factoryDefaults();
    driveMotor.factoryDefaults();

    // Configure voltage comp, current limit, and ramp rate.
    angleMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    driveMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    angleMotor.setCurrentLimit(configuration.physicalCharacteristics.angleMotorCurrentLimit);
    driveMotor.setCurrentLimit(configuration.physicalCharacteristics.driveMotorCurrentLimit);
    angleMotor.setLoopRampRate(configuration.physicalCharacteristics.angleMotorRampRate);
    driveMotor.setLoopRampRate(configuration.physicalCharacteristics.driveMotorRampRate);

    // Config angle encoders
    absoluteEncoder = moduleConfiguration.absoluteEncoder;
    if (absoluteEncoder != null)
    {
      absoluteEncoder.factoryDefault();
      absoluteEncoder.configure(moduleConfiguration.absoluteEncoderInverted);
      angleMotor.setPosition(getAbsolutePosition());
    }

    // Config angle motor/controller
    angleMotor.configureIntegratedEncoder(
        moduleConfiguration.getPositionEncoderConversion(false));
    angleMotor.configurePIDF(moduleConfiguration.anglePIDF);
    angleMotor.configurePIDWrapping(-180, 180);
    angleMotor.setInverted(moduleConfiguration.angleMotorInverted);
    angleMotor.setMotorBrake(false);

    // Config drive motor/controller
    driveMotor.configureIntegratedEncoder(moduleConfiguration.getPositionEncoderConversion(true));
    driveMotor.configurePIDF(moduleConfiguration.velocityPIDF);
    driveMotor.setInverted(moduleConfiguration.driveMotorInverted);
    driveMotor.setMotorBrake(true);

    driveMotor.burnFlash();
    angleMotor.burnFlash();

    if (SwerveDriveTelemetry.isSimulation)
    {
      simModule = new SwerveModuleSimulation();
    }

    lastAngle = getState().angle.getDegrees();
    lastVelocity = getState().speedMetersPerSecond;
  }

  /**
   * Queue synchronization of the integrated angle encoder with the absolute encoder.
   */
  public void queueSynchronizeEncoders()
  {
    if (absoluteEncoder != null)
    {
      synchronizeEncoderQueued = true;
    }
  }

  /**
   * Set the desired state of the swerve module. <br /><b>WARNING: If you are not using one of the functions from
   * {@link SwerveDrive} you may screw up {@link SwerveDrive#kinematics}</b>
   *
   * @param desiredState Desired swerve module state.
   * @param isOpenLoop   Whether to use open loop (direct percent) or direct velocity control.
   * @param force        Disables optimizations that prevent movement in the angle motor and forces the desired state
   *                     onto the swerve module.
   */
  public void setDesiredState(SwerveModuleState2 desiredState, boolean isOpenLoop, boolean force)
  {
    SwerveModuleState simpleState =
        new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    simpleState = SwerveModuleState.optimize(simpleState, getState().angle);
    desiredState =
        new SwerveModuleState2(
            simpleState.speedMetersPerSecond, simpleState.angle, desiredState.omegaRadPerSecond);

    if (isOpenLoop)
    {
      double percentOutput = desiredState.speedMetersPerSecond / configuration.maxSpeed;
      driveMotor.set(percentOutput);
    } else
    {
      double velocity = desiredState.speedMetersPerSecond;
      if (velocity != lastVelocity)
      {
        driveMotor.setReference(velocity, feedforward.calculate(velocity));
      }
      lastVelocity = velocity;
    }

    double angle = desiredState.angle.getDegrees();

    // If we are forcing the angle
    if (!force)
    {
      // Prevents module rotation if speed is less than 1%
      angle = Math.abs(desiredState.speedMetersPerSecond) <= (configuration.maxSpeed * 0.01) ? lastAngle : angle;
    }

    // Ensure the angle is above 0
    while (angle < 0)
    {
      angle += 360;
    }

    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      SmartDashboard.putNumber(
          "Optimized " + moduleNumber + " Speed Setpoint: ", desiredState.speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Optimized " + moduleNumber + " Angle Setpoint: ", angle);
      SmartDashboard.putNumber(
          "Module " + moduleNumber + " Omega: ", Math.toDegrees(desiredState.omegaRadPerSecond));
    }

    // Prevent module rotation if angle is the same as the previous angle.
    if (angle != lastAngle || synchronizeEncoderQueued)
    {
      // Synchronize encoders if queued and send in the current position as the value from the absolute encoder.
      if (absoluteEncoder != null && synchronizeEncoderQueued)
      {
        double absoluteEncoderPosition = getAbsolutePosition();
        angleMotor.setPosition(absoluteEncoderPosition);
        angleMotor.setReference(angle,
                                Math.toDegrees(desiredState.omegaRadPerSecond) * configuration.angleKV,
                                absoluteEncoderPosition);
        synchronizeEncoderQueued = false;
      } else
      {
        angleMotor.setReference(angle, Math.toDegrees(desiredState.omegaRadPerSecond) * configuration.angleKV);
      }
    }
    lastAngle = angle;

    if (SwerveDriveTelemetry.isSimulation)
    {
      simModule.updateStateAndPosition(desiredState);
    }
  }

  /**
   * Set the angle for the module.
   *
   * @param angle Angle in degrees.
   */
  public void setAngle(double angle)
  {
    angleMotor.setReference(angle, configuration.angleKV);
    lastAngle = angle;
  }

  /**
   * Get the Swerve Module state.
   *
   * @return Current SwerveModule state.
   */
  public SwerveModuleState2 getState()
  {
    double     velocity;
    Rotation2d azimuth;
    double     omega;
    if (!SwerveDriveTelemetry.isSimulation)
    {
      velocity = driveMotor.getVelocity();
      azimuth = Rotation2d.fromDegrees(angleMotor.getPosition());
      omega = Math.toRadians(angleMotor.getVelocity());
    } else
    {
      return simModule.getState();
    }
    return new SwerveModuleState2(velocity, azimuth, omega);
  }

  /**
   * Get the position of the swerve module.
   *
   * @return {@link SwerveModulePosition} of the swerve module.
   */
  public SwerveModulePosition getPosition()
  {
    double     position;
    Rotation2d azimuth;
    if (!SwerveDriveTelemetry.isSimulation)
    {
      position = driveMotor.getPosition();
      azimuth = Rotation2d.fromDegrees(angleMotor.getPosition());
    } else
    {
      return simModule.getPosition();
    }
    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      SmartDashboard.putNumber("Module " + moduleNumber + "Angle", azimuth.getDegrees());
    }
    return new SwerveModulePosition(position, azimuth);
  }

  /**
   * Get the absolute position. Falls back to relative position on reading failure.
   *
   * @return Absolute encoder angle in degrees in the range [0, 360).
   */
  public double getAbsolutePosition()
  {
    double angle;
    if (absoluteEncoder != null)
    {
      angle = absoluteEncoder.getAbsolutePosition() - angleOffset;
      if (absoluteEncoder.readingError)
      {
        angle = getRelativePosition();
      }
    } else
    {
      angle = getRelativePosition();
    }
    angle %= 360;
    if (angle < 0.0)
    {
      angle += 360;
    }

    return angle;
  }

  /**
   * Get the relative angle in degrees.
   *
   * @return Angle in degrees.
   */
  public double getRelativePosition()
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

  /**
   * Get the angle {@link SwerveMotor} for the {@link SwerveModule}.
   *
   * @return {@link SwerveMotor} for the angle/steering motor of the module.
   */
  public SwerveMotor getAngleMotor()
  {
    return angleMotor;
  }

  /**
   * Get the drive {@link SwerveMotor} for the {@link SwerveModule}.
   *
   * @return {@link SwerveMotor} for the drive motor of the module.
   */
  public SwerveMotor getDriveMotor()
  {
    return driveMotor;
  }

  /**
   * Fetch the {@link SwerveModuleConfiguration} for the {@link SwerveModule} with the parsed configurations.
   *
   * @return {@link SwerveModuleConfiguration} for the {@link SwerveModule}.
   */
  public SwerveModuleConfiguration getConfiguration()
  {
    return configuration;
  }
}

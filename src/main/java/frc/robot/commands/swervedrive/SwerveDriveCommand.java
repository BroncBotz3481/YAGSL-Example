package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveDriveCommand extends CommandBase
{

  private final SwerveSubsystem  swerveSubsystem;
  private final Supplier<Double> strafeSpdFunction, forwardSpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  public SwerveDriveCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> strafeSpdFunction,
                            Supplier<Double> forwardSpdFunction, Supplier<Double> turningSpdFunction,
                            Supplier<Boolean> fieldOrientedFunction)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.strafeSpdFunction = strafeSpdFunction;
    this.forwardSpdFunction = forwardSpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    // 1. Get real-time joystick inputs
    double strafe       = strafeSpdFunction.get();
    double forward      = forwardSpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    swerveSubsystem.drive(forward, strafe, turningSpeed, !fieldOrientedFunction.get());
    /*


    // 2. Apply deadband/Dead-Zone
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // 3. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if ()
    {
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
                                                            swerveSubsystem.getRotation2d());
    } else
    {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);

     */
  }

  @Override
  public void end(boolean interrupted)
  {
    swerveSubsystem.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
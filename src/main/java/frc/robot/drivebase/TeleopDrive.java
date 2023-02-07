// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivebase;

import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  private SwerveBase swerve;
  private DoubleSupplier vX, vY, omega;
  private BooleanSupplier driveMode;
  private boolean isOpenLoop;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(SwerveBase swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode, boolean isOpenLoop) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.isOpenLoop = isOpenLoop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = Math.pow(vX.getAsDouble(), 3) * Drivebase.MAX_SPEED;
    double yVelocity = Math.pow(vY.getAsDouble(), 3) * Drivebase.MAX_SPEED;
    double angVelocity = Math.pow(omega.getAsDouble(), 3) * Drivebase.MAX_ANGULAR_VELOCITY;
    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);
    swerve.drive(
      new Translation2d(
        xVelocity,
        yVelocity),
      angVelocity,
      driveMode.getAsBoolean(),
      isOpenLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

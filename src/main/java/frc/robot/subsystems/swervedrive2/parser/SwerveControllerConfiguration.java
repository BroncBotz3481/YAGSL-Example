package frc.robot.subsystems.swervedrive2.parser;

import static frc.robot.subsystems.swervedrive2.math.SwerveMath.calculateMaxAngularVelocity;

import frc.robot.Constants.Drivebase;
import frc.robot.Constants.Drivebase.DrivetrainLimitations;
import frc.robot.Constants.Drivebase.ModuleLocations;

public class SwerveControllerConfiguration
{

  public final double     maxSpeed           = DrivetrainLimitations.MAX_SPEED;
  public final double     maxAngularVelocity = calculateMaxAngularVelocity(maxSpeed,
                                                                           ModuleLocations.FRONT_LEFT_X,
                                                                           ModuleLocations.BACK_LEFT_Y);
  public       double     hypotDeadband      = 0.5; // Deadband for the minimum hypot for the heading joystick.
  public       PIDFConfig headingPIDF        = new PIDFConfig(Drivebase.HEADING_KP, Drivebase.HEADING_KI,
                                                              Drivebase.HEADING_KD);

}

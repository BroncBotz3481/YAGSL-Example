package frc.robot.subsystems.swervedrive2.parser;

import frc.robot.Constants.Drivebase;
import frc.robot.Constants.Drivebase.DrivetrainLimitations;

public class SwerveControllerConfiguration
{

  public final double     maxSpeed           = DrivetrainLimitations.MAX_SPEED;
  public final double     maxAngularVelocity = DrivetrainLimitations.MAX_ANGULAR_VELOCITY;
  public       double     hypotDeadband      = 0.5; // Deadband for the minimum hypot for the heading joystick.
  public       PIDFConfig headingPIDF        = new PIDFConfig(Drivebase.HEADING_KP, Drivebase.HEADING_KI,
                                                              Drivebase.HEADING_KD);

}

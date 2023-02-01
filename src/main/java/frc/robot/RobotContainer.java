// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.Auto1Command;
import frc.robot.commands.swervedrive.SwerveDriveCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  public final  SwerveSubsystem swerveSubsystem  = new SwerveSubsystem();
  private final XboxController  driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Entire program modelled off of https://github.com/SeanSun6814/FRC0ToAutonomous/tree/master/%236%20Swerve%20Drive%20Auto
    // Renamed JoystickSwerveCmd to SwerveDriveCommand
    swerveSubsystem.setDefaultCommand(
        new SwerveDriveCommand(swerveSubsystem, driverController::getLeftX,
                               driverController::getLeftY,
                               driverController::getRightX,
                               () -> driverController.getAButton()));
    /// Configure the button bindings
//    controller0 = new XboxController(0);
//    controller1 = new XboxController(1);
    configureButtonBindings();
  }

  private void configureButtonBindings()
  {
//    new JoystickButton(driverController, 2).whenPressed(swerveSubsystem.m_drive::zeroGyro); // NEW
  }

  public Command getAutonomousCommand()
  {
    Auto1Command auto1Command = new Auto1Command(swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.m_drive.resetOdometry(auto1Command.getInitialPosition())), //Import?
        auto1Command, new InstantCommand(() -> swerveSubsystem.m_drive.stopMotor()));

//    return null;
  }
}
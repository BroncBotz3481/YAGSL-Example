// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  /*
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                             .setKinematics(
        DriveConstants.kDriveKinematics);

    // 2. Generate trajectory ---Is all the Poses put together in Autonomous
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(new Translation2d(1, 0),
                                                                           new Translation2d(1, -1)),
                                                                   new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                                                                   trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                                                      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    // Changed to use the wpilib SwerveControllerCommand
    // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command
    /SwerveControllerCommand.html
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose,
                                                                                  DriveConstants.kDriveKinematics,
                                                                                  xController, yController,
                                                                                  thetaController,
                                                                                  swerveSubsystem::setModuleStates,
                                                                                  swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), //Import?
        swerveControllerCommand, new InstantCommand(() -> swerveSubsystem.stopModules()));
    */
    return null;
  }
}
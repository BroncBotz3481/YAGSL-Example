// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // POV triggers quick rotation positions to different ways to face
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1,
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1,
                                                                                               OperatorConstants.LEFT_X_DEADBAND),
                                                                 () -> MathUtil.applyDeadband(driverXbox.getRightX() * -1,
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 ()-> (driverXbox.getHID().getPOV() == 0),
                                                                 ()-> (driverXbox.getHID().getPOV() == 180),
                                                                 ()-> (driverXbox.getHID().getPOV() == 90),
                                                                 ()-> (driverXbox.getHID().getPOV() == 270));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  // This command prevents heading change when the comand starts
  AbsoluteDrive absoluteDrive = new AbsoluteDrive(drivebase,
                                                                 () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1,
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1,
                                                                                               OperatorConstants.LEFT_X_DEADBAND),
                                                                 () -> MathUtil.applyDeadband(driverXbox.getRightX() * -1,
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 () -> MathUtil.applyDeadband(driverXbox.getRightY() * -1,
                                                                                               OperatorConstants.RIGHT_X_DEADBAND));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX() * -1,
      () -> driverXbox.getRightY() * -1);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX() * -1);

  Command driveSetpointGen = drivebase.driveWithSetpointGenerator(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));

  Command driveSetpointGenSim = drivebase.driveWithSetpointGenerator(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));

  // Create a chooser to allow selecting the drive mode from a dashboard
  enum DriveMode {
    ABSOLUTE_DRIVE,
    ABSOLUTE_ADVANCED,
    DIRECT_ANGLE,
    ANGULAR_VELOCITY,
    SET_POINT_GEN
  }

  // Drive mode chooser to allow changing mode each time TeleOp is enabled. Default is used if 
  // chooser is not opened.
  // Pull up on dashboard or sim GUI (SmartDashborad/SendableChooser[0])
  SendableChooser<DriveMode> driveChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Setup chooser for selecting drive mode and set the default mode
    driveChooser.setDefaultOption("Drive Mode - Direct Angle", DriveMode.DIRECT_ANGLE);
    driveChooser.addOption("Drive Mode - Angular Velocity", DriveMode.ANGULAR_VELOCITY);
    driveChooser.addOption("Drive Mode - Absolute Drive", DriveMode.ABSOLUTE_DRIVE);
    driveChooser.addOption("Drive Mode - Absolute Advanced", DriveMode.ABSOLUTE_ADVANCED);
    driveChooser.addOption("Drive Mode - Set Point Gen", DriveMode.SET_POINT_GEN);
    SmartDashboard.putData(driveChooser);

    setDriveMode();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
      driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  /**
   * Use this to set the drive mode. This should be called in testInit and teleopInit in the {@link Robot} class.
   * The mode change will then take effect when the robot is enabled.
   */
  public void setDriveMode()
  {
    switch (driveChooser.getSelected()) {

      case ANGULAR_VELOCITY:
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        return;

      case ABSOLUTE_DRIVE:
        drivebase.setDefaultCommand(absoluteDrive);
        return;
        
      case ABSOLUTE_ADVANCED:
        drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
        return;

      case SET_POINT_GEN:
        drivebase.setDefaultCommand(driveSetpointGen);
        return;

      case DIRECT_ANGLE:
      default:
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    }
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

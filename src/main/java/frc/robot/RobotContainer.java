// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.drivebase.AbsoluteDrive;
import frc.robot.drivebase.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveBase drivebase = new SwerveBase();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private SendableChooser<CommandBase> driveModeSelector;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(OperatorConstants.DRIVER_CONTROLLER_PORT);
  CommandJoystick rotationController = new CommandJoystick(1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive absoluteDrive = new AbsoluteDrive(
      drivebase,
      // Applies deadbands and inverts controls because joysticks are back-right positive while robot
      // controls are front-left positive
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -rotationController.getX(),
      () -> -rotationController.getY(), true);

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(
      drivebase,
      // Applies deadbands and inverts controls because joysticks are back-right positive while robot
      // controls are front-left positive
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -rotationController.getX(),
      () -> -rotationController.getY(), false);

    TeleopDrive openRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> false, true);
    
    TeleopDrive closedRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> false, false);
    
    TeleopDrive openFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> true, true);

    TeleopDrive closedFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> true, false);

    driveModeSelector = new SendableChooser<>();
    driveModeSelector.setDefaultOption("AbsoluteDrive", absoluteDrive);
    driveModeSelector.addOption("Field Relative", openFieldRel);
    driveModeSelector.addOption("Robot Relative", openRobotRel);
    driveModeSelector.addOption("Absolute (Closed)", closedAbsoluteDrive);
    driveModeSelector.addOption("Field Relative (Closed)", closedFieldRel);
    driveModeSelector.addOption("Robot Relative (Closed)", closedRobotRel);

    SmartDashboard.putData(driveModeSelector);
    drivebase.setDefaultCommand(openFieldRel);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    driverController.button(1).onTrue((new InstantCommand(drivebase::zeroGyro)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.driveAndSpin(drivebase);
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}

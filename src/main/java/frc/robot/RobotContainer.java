// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoMap;
import frc.robot.commands.swervedrive.auto.GoToPose;
import frc.robot.commands.swervedrive.auto.GoToScoring;
import frc.robot.commands.swervedrive.auto.PathBuilder;
import frc.robot.commands.swervedrive.auto.GoToScoring.POSITION;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private SendableChooser<Command> chooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));
  //private final Shooter shooter = new Shooter();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private static final CommandXboxController driverXbox = new CommandXboxController(0);
  private static final CommandXboxController operatorXbox = new CommandXboxController(1);


  private final AutoMap autoMap = new AutoMap();
  private final PathBuilder builder = new PathBuilder(drivebase, autoMap.getEventMap());

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;


  private final TeleopDrive closedFieldRel =
  new TeleopDrive(
      drivebase,
      () -> -driverXbox.getRawAxis(translationAxis), 
      () -> -driverXbox.getRawAxis(strafeAxis), 
      () -> -driverXbox.getRawAxis(rotationAxis), 
      () -> driverXbox.leftBumper().getAsBoolean(),
      false,
      false
      );
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    initializeChooser();

    drivebase.setDefaultCommand(closedFieldRel);
    

    //shooter.setDefaultCommand(shooter.moveArm(() -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis()));
  }

  private void initializeChooser() {

    chooser.addOption(
        "Default Test",
        builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "Test Path", new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION))));

    chooser.addOption(
      "Calibration 2 Meter",
      builder.getSwerveCommand(
          PathPlanner.loadPathGroup(
              "Calibration Test", new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION))));

    chooser.setDefaultOption(
        "Test - 2", 
        builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "SamplePath", new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION))));

    chooser.addOption(
          "Shoot High And Balance - No PathPlanner",
          drivebase.moveRevOntoChargeStation()
          .andThen(() -> drivebase.balanceRobot()));

    SmartDashboard.putData("Auto", chooser);
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
    driverXbox.a().whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    driverXbox.y().onTrue((new InstantCommand(drivebase::zeroGyro)));



    operatorXbox.povRight().whileTrue(new GoToScoring(drivebase, POSITION.RIGHT).getCommand());
    operatorXbox.povDown().whileTrue(new GoToScoring(drivebase, POSITION.MIDDLE).getCommand());
    operatorXbox.povLeft().whileTrue(new GoToScoring(drivebase, POSITION.LEFT).getCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return chooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;

public final class Autos
{

  private Autos()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase exampleAuto(SwerveSubsystem swerve)
  {
    PathPlannerTrajectory example = PathPlanner.loadPath("New Path",
                                                         new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION));
    return Commands.sequence(new FollowTrajectory(swerve, example, true));
  }

  public static CommandBase driveAndSpin(SwerveSubsystem swerve)
  {
    return Commands.sequence(
        new RepeatCommand(new InstantCommand(() -> swerve.drive(new Translation2d(1, 0), 5, true, true), swerve)));
  }
}

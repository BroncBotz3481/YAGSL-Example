// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Auton;
import frc.robot.autoCommands.FollowTrajectory;
import frc.robot.subsystems.SwerveBase;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(SwerveBase swerve) {
    PathPlannerTrajectory example = PathPlanner.loadPath("New Path", 
      new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION));
    return Commands.sequence(new FollowTrajectory(swerve, example, true));
  }

  public static CommandBase driveAndSpin(SwerveBase swerve) {
    return Commands.sequence(new RepeatCommand(new InstantCommand(() -> swerve.drive(new Translation2d(1, 0), 5, true, true), swerve)));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.Optional;

public class GoToScoring {
  private final SwerveSubsystem drive;

  public enum POSITION {
    LEFT,
    MIDDLE,
    RIGHT
  }

  public GoToScoring(SwerveSubsystem drive) {
    this.drive = drive;
  }

  /**
   * Get best scoring area. Assumes scoring area zones do not overlap.
   *
   * @param pose current pose of robot
   * @return either null if not in scoring area, or the scoring are if in scoring area
   */
  public Command getCommand(Pose2d pose) {
    System.out.println("Scoring Position Scheduled");
    Command command;
    GoToPose goToPose = new GoToPose(
                                new Pose2d(new Translation2d(9.91, 6.82), new Rotation2d(Math.toRadians(0))),
                                new PathConstraints(5, 3),
                                drive);
    command = goToPose.getCommand();


    System.out.println(command.toString());
    return command;
  }

  public Command getCommand() {
    return new ProxyCommand(() -> getCommand(drive.getPose()))
        .andThen(Commands.waitSeconds(1));
  }
}
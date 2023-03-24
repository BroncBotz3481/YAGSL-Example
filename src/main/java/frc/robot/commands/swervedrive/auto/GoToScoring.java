package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.Optional;
import webblib.util.chargedup.ScoringArea;

public class GoToScoring {
  private final SwerveSubsystem drive;
  private final List<ScoringArea> scoreAreaList = Auton.scoreAreaList;
  private final POSITION selectedPosition;

  public enum POSITION {
    LEFT,
    MIDDLE,
    RIGHT
  }

  public GoToScoring(SwerveSubsystem drive, POSITION selectedPosition) {
    this.drive = drive;
    this.selectedPosition = selectedPosition;
  }

  /**
   * Get best scoring area. Assumes scoring area zones do not overlap.
   *
   * @param pose current pose of robot
   * @return either null if not in scoring area, or the scoring are if in scoring area
   */
  private Optional<ScoringArea> getBestScoringArea(Pose2d pose) {
    ScoringArea bestArea = null;
    for (ScoringArea area : scoreAreaList) {
      if (area.isPoseWithinScoringArea(pose)) bestArea = area;
    }
    if (bestArea == null) {
      return Optional.empty();
    }
    return Optional.of(bestArea);
  }

  public Command getCommand(Pose2d pose) {
    System.out.println("Scoring Position Scheduled");
    Optional<ScoringArea> scoringArea = getBestScoringArea(pose);
    Command command;
    if (!scoringArea.isEmpty()) {
      GoToPose goToPose;
      switch (selectedPosition) {
        case LEFT:
          goToPose =
              new GoToPose(
                  scoringArea.get().getLeftPosition().getPoseMeters(),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        case MIDDLE:
          goToPose =
              new GoToPose(
                  scoringArea.get().getMiddlePosition().getPoseMeters(),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        case RIGHT:
          goToPose =
              new GoToPose(
                  scoringArea.get().getRightPosition().getPoseMeters(),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Unsupported scoring enum");
      }
    } else {
      command = Commands.none();
    }
    System.out.println(command.toString());
    return command;
  }

  public Command getCommand() {
    return new ProxyCommand(() -> getCommand(drive.getPose()))
        .andThen(Commands.waitSeconds(1));
  }
}

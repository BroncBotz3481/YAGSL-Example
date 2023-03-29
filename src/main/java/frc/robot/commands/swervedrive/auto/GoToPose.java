package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;

public class GoToPose {
  private PPSwerveControllerCommand ppSwerveCommand;
  private PathPlannerTrajectory traj;

  public GoToPose(
      Pose2d pose, Rotation2d heading, PathConstraints constraints, SwerveSubsystem drive) {
    Translation2d translation = pose.getTranslation();
    Rotation2d holonomic = pose.getRotation();

    PathPoint currentPathPoint;
    if (Math.hypot(
            drive.getFieldVelocity().vxMetersPerSecond, drive.getFieldVelocity().vyMetersPerSecond)
        > 0.2) {
      currentPathPoint =
          PathPoint.fromCurrentHolonomicState(drive.getPose(), drive.getFieldVelocity());
    } else {
      currentPathPoint =
          new PathPoint(
              drive.getPose().getTranslation(),
              translation.minus(drive.getPose().getTranslation()).getAngle(),
              drive.getPose().getRotation());
    }

    List<PathPoint> path =
        new ArrayList<PathPoint>() {
          {
            add(currentPathPoint);
            add(new PathPoint(translation, heading, holonomic));
          }
        };

    traj = PathPlanner.generatePath(constraints, path);
    // position, heading(direction of travel), holonomic rotation

    ppSwerveCommand =
        new PPSwerveControllerCommand(
            traj,
            drive::getPose, // Pose supplier
            Auton.xAutoPID.createPIDController(),
            Auton.yAutoPID.createPIDController(),
            Auton.angleAutoPID.createPIDController(),
            // Rotation controller.
            drive::setChassisSpeeds, // Module states consumer
            true,
            drive // Requires this drive subsystem
            );
  }

  private PathConstraints calculateRotationalConstraints(
      Transform2d transform, PathConstraints constraints) {
    var maxRadPerSecond = 2.0;

    var desiredTime = 1 / maxRadPerSecond * transform.getRotation().getRadians();
    var updatedConstraints = 1 / (desiredTime / transform.getTranslation().getNorm());

    return new PathConstraints(Math.abs(updatedConstraints), constraints.maxAcceleration);
  }

  public GoToPose(Pose2d pose, PathConstraints constraints, SwerveSubsystem drive) {
    Translation2d translation = pose.getTranslation();
    Rotation2d holonomic = pose.getRotation();
    var heading = translation.minus(drive.getPose().getTranslation()).getAngle();
    var diffPose = pose.minus(drive.getPose());

    PathPoint currentPathPoint;
    if (Math.hypot(
            drive.getFieldVelocity().vxMetersPerSecond, drive.getFieldVelocity().vyMetersPerSecond)
        > 0.2) {
      currentPathPoint =
          PathPoint.fromCurrentHolonomicState(drive.getPose(), drive.getFieldVelocity());
    } else {
      currentPathPoint =
          new PathPoint(drive.getPose().getTranslation(), heading, drive.getPose().getRotation());
    }

    List<PathPoint> path =
        new ArrayList<PathPoint>() {
          {
            add(currentPathPoint);
            add(new PathPoint(translation, heading, holonomic));
          }
        };

    if (Math.abs(diffPose.getTranslation().getNorm()) < 0.05
        && Math.abs(diffPose.getRotation().getRadians()) > 0.5) {
      constraints = calculateRotationalConstraints(diffPose, constraints);
    }
    traj = PathPlanner.generatePath(constraints, path);
    ppSwerveCommand = drive.followTrajectoryCommand(traj);
    // ppSwerveCommand =
    //     new PPSwerveControllerCommand(
    //         traj,
    //         drive::getPose, // Pose supplier
    //         Auton.xAutoPID.createPIDController(),
    //         Auton.yAutoPID.createPIDController(),
    //         Auton.angleAutoPID.createPIDController(),
    //         drive::setChassisSpeeds,
    //         true,
    //         drive // Requires this drive subsystem
    //         );
  }

  public PPSwerveControllerCommand getCommand() {
    return ppSwerveCommand;
  }
}
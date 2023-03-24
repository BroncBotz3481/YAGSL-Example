package webblib.util;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class HolonomicPose2d {
  private final Pose2d pose2d;
  private final Rotation2d heading;

  /**
   * Create a holonomic Pose2d to be used with trajectories. Includes an actual pose of the robot,
   * where rotation is the robot's actual rotation about itself, and a heading, where the robot is
   * going to go.
   *
   * @param pose2d the Pose2d of the robot.
   * @param heading the heading of the robot (where should it go?).
   */
  public HolonomicPose2d(Pose2d pose2d, Rotation2d heading) {
    this.pose2d = pose2d;
    this.heading = heading;
  }

  /**
   * @return pose of the robot
   */
  public Pose2d getPoseMeters() {
    return pose2d;
  }

  /**
   * @return desired heading
   */
  public Rotation2d getRotation() {
    return heading;
  }

  /**
   * @return preconstructed PathPoint with pose and heading
   */
  public PathPoint getPathPoint() {
    return new PathPoint(pose2d.getTranslation(), heading, pose2d.getRotation());
  }
}

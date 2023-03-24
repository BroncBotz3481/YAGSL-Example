package webblib.util.chargedup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import webblib.util.HolonomicPose2d;
import webblib.util.RectanglePoseArea;

public class ScoringArea {
  private final RectanglePoseArea scoreRect;
  private final HolonomicPose2d leftPosition;
  private final HolonomicPose2d middlePosition;
  private final HolonomicPose2d rightPosition;

  public ScoringArea(
      RectanglePoseArea scoreRect,
      HolonomicPose2d leftPosition,
      HolonomicPose2d middlePosition,
      HolonomicPose2d rightPosition) {
    this.scoreRect = scoreRect;
    this.leftPosition = leftPosition;
    this.middlePosition = middlePosition;
    this.rightPosition = rightPosition;
  }

  public RectanglePoseArea getScoreRectangle() {
    return scoreRect;
  }

  public HolonomicPose2d getLeftPosition() {
    return DriverStation.getAlliance() == Alliance.Blue ? leftPosition : rightPosition;
  }

  public HolonomicPose2d getMiddlePosition() {
    return middlePosition;
  }

  public HolonomicPose2d getRightPosition() {
    return DriverStation.getAlliance() == Alliance.Blue ? rightPosition : leftPosition;
  }

  public boolean isPoseWithinScoringArea(Pose2d pose) {
    return scoreRect.isPoseWithinArea(pose);
  }

  public double getDistanceFromPose(Pose2d pose) {
    var dx =
        Math.max(scoreRect.getMinX() - pose.getX(), Math.max(0, pose.getX() - scoreRect.getMaxX()));
    var dy =
        Math.max(scoreRect.getMinY() - pose.getY(), Math.max(0, pose.getY() - scoreRect.getMaxY()));
    return Math.sqrt(dx * dx + dy * dy);
  }
}

package webblib.util.chargedup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import webblib.util.HolonomicPose2d;
import webblib.util.RectanglePoseArea;

public class LoadingArea {
  private final RectanglePoseArea largeLoadingRect;
  private final RectanglePoseArea smallLoadingRect;
  private final HolonomicPose2d doubleSubstationLeft;
  private final HolonomicPose2d doubleSubstationRight;

  public LoadingArea(
      RectanglePoseArea largeLoadingRect,
      RectanglePoseArea smallLoadingRect,
      HolonomicPose2d doubleSubstationLeft,
      HolonomicPose2d doubleSubstationRight) {
    this.largeLoadingRect = largeLoadingRect;
    this.smallLoadingRect = smallLoadingRect;
    this.doubleSubstationLeft = doubleSubstationLeft;
    this.doubleSubstationRight = doubleSubstationRight;
  }

  public RectanglePoseArea getLargeLoadingRectangle() {
    return largeLoadingRect;
  }

  public RectanglePoseArea getSmallLoadingRectangle() {
    return smallLoadingRect;
  }

  public HolonomicPose2d getDoubleSubstationLeft() {
    return DriverStation.getAlliance() == Alliance.Blue
        ? doubleSubstationLeft
        : doubleSubstationRight;
  }

  public HolonomicPose2d getDoubleSubstationRight() {
    return DriverStation.getAlliance() == Alliance.Blue
        ? doubleSubstationRight
        : doubleSubstationLeft;
  }

  public boolean isPoseWithinScoringArea(Pose2d pose) {
    return largeLoadingRect.isPoseWithinArea(pose) || smallLoadingRect.isPoseWithinArea(pose);
  }
}

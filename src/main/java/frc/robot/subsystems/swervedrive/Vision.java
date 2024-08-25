package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;


/**
 * Example Vision class to aid in the pursuit of accurate odometry, using PhotonVision. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision
{

  /**
   * Count of times that the odom thinks we're more than 10meters away from the april tag.
   */
  private double longDistangePoseEstimationCount = 0;

  /**
   * Left PhotonCamera instance.
   */
  private PhotonCamera leftCam;
  /**
   * Right PhotonCamera instance.
   */
  private PhotonCamera rightCam;
  /**
   * Center Photon Camera instance
   */
  private PhotonCamera centerCam;

  /**
   * April Tag Field Layout of the year.
   */
  private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  /**
   * Left Camera Photon Pose Estimator
   */
  private PhotonPoseEstimator leftPoseEstimator;
  /**
   * Right Camera Photon Pose Estimator
   */
  private PhotonPoseEstimator rightPoseEstimator;
  /**
   * Center Camera Photon Pose Estimator
   */
  private PhotonPoseEstimator centerPoseEstimator;

  /**
   * Left camera relative to the center of the robot.
   */
  private final Translation3d leftRobotToCamTranslation = new Translation3d(Units.inchesToMeters(12.056),
                                                                            Units.inchesToMeters(10.981),
                                                                            Units.inchesToMeters(8.44));
  /**
   * Left camera relative to the center of the robot rotationally.
   */
  private final Rotation3d    leftRobotToCamRotation    = new Rotation3d(0,
                                                                         Math.toRadians(-24.094),
                                                                         Math.toRadians(30));

  /**
   * Right camera relative to the center of the robot.
   */
  private final Translation3d rightRobotToCamTranslation  = new Translation3d(Units.inchesToMeters(12.056),
                                                                              Units.inchesToMeters(-10.981),
                                                                              Units.inchesToMeters(8.44));
  /**
   * Right camera relative to the center of the robot rotationally.
   */
  private final Rotation3d    rightRobotToCamRotation     = new Rotation3d(0,
                                                                           Math.toRadians(-24.094),
                                                                           Math.toRadians(-30));
  /**
   * Center camera relative to the center of the robot.
   */
  private final Translation3d centerRobotToCamTranslation = new Translation3d(Units.inchesToMeters(-4.628),
                                                                              Units.inchesToMeters(-10.687),
                                                                              Units.inchesToMeters(16.129));
  /**
   * Center camera relative to the center of the robot rotationally.
   */
  private final Rotation3d    centerRobotToCamRotation    = new Rotation3d(0, Units.degreesToRadians(18), 0);

  /**
   * Photon Vision Simulation
   */
  private VisionSystemSim     visionSim;
  /**
   * Photon Vision camera properties simulation.
   */
  private SimCameraProperties cameraProp;
  /**
   * Left camera for simulation.
   */
  private PhotonCameraSim     simLeftCam;
  /**
   * Right Camera for simulation.
   */
  private PhotonCameraSim     simRightCam;
  /**
   * Center camera for simulation
   */
  private PhotonCameraSim     simCenterCam;

  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d          field2d;

  /**
   * Left camera latency Alert.
   */
  public Alert leftCamLatancyAlert  = new Alert("Left Camera is experiencing high latency", AlertType.WARNING);
  /**
   * Right camera latency Alert.
   */
  public Alert rightCamLatancyAlert = new Alert("Right Camera is experiencing high latency", AlertType.WARNING);

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field)
  {
    this.currentPose = currentPose;
    this.field2d = field;

    leftCam = new PhotonCamera("left");
    rightCam = new PhotonCamera("right");
    centerCam = new PhotonCamera("center");

    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    Transform3d leftRobotToCam   = new Transform3d(leftRobotToCamTranslation, leftRobotToCamRotation);
    Transform3d rightRobotToCam  = new Transform3d(rightRobotToCamTranslation, rightRobotToCamRotation);
    Transform3d centerRobotToCam = new Transform3d(centerRobotToCamTranslation, centerRobotToCamRotation);

    leftPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                leftCam,
                                                leftRobotToCam);
    rightPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                                                 PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                 rightCam,
                                                 rightRobotToCam);
    centerPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                                                  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                  centerCam,
                                                  centerRobotToCam);

    leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    centerPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      cameraProp = new SimCameraProperties();
      // A 640 x 480 camera with a 100 degree diagonal FOV.
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
      // Approximate detection noise with average and standard deviation error in pixels.
      cameraProp.setCalibError(0.25, 0.08);
      // Set the camera image capture framerate (Note: this is limited by robot loop rate).
      cameraProp.setFPS(30);
      // The average and standard deviation in milliseconds of image data latency.
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      simLeftCam = new PhotonCameraSim(leftCam, cameraProp);
      simLeftCam.enableDrawWireframe(true);

      simRightCam = new PhotonCameraSim(rightCam, cameraProp);
      simRightCam.enableDrawWireframe(true);

      simCenterCam = new PhotonCameraSim(centerCam, cameraProp);
      simCenterCam.enableDrawWireframe(true);

      visionSim.addCamera(simLeftCam, leftRobotToCam);
      visionSim.addCamera(simRightCam, rightRobotToCam);
      visionSim.addCamera(simCenterCam, centerRobotToCam);

      openSimCameraViews();
    }
  }


  /**
   * generates the estimated robot pose. Returns empty if:
   * <ul>
   *  <li> No Pose Estimates could be generated</li>
   * <li> The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
   */
  public ArrayList<EstimatedRobotPose> getEstimatedGlobalPose()
  {
    Optional<EstimatedRobotPose> leftPoseEst  = filterPose(leftPoseEstimator.update());
    Optional<EstimatedRobotPose> rightPoseEst = filterPose(rightPoseEstimator.update());
    //Optional<EstimatedRobotPose> centerPoseEst = Optional.empty();

    ArrayList<EstimatedRobotPose> poses = new ArrayList<>();
    if (leftPoseEst.isPresent())
    {
      poses.add(leftPoseEst.get());
      field2d.getObject("Left est pose").setPose(leftPoseEst.get().estimatedPose.toPose2d());
    }

    if (rightPoseEst.isPresent())
    {
      poses.add(rightPoseEst.get());
      field2d.getObject("Right est pose").setPose(rightPoseEst.get().estimatedPose.toPose2d());
    }

    return poses;
  }

  /**
   * Filter pose via the ambiguity and find best estimate between all of the camera's throwing out distances more than
   * 10m for a short amount of time.
   *
   * @param pose Estimated robot pose.
   * @return Could be empty if there isn't a good reading.
   */
  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose)
  {
    if (pose.isPresent())
    {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed)
      {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity)
        {
          bestTargetAmbiguity = ambiguity;
        }
      }
      //ambiguity to high dont use estimate
      if (bestTargetAmbiguity > 0.3)
      {
        return Optional.empty();
      }

      //est pose is very far from recorded robot pose
      if (getDistanceFromPose(pose.get().estimatedPose.toPose2d()) > 1)
      {
        longDistangePoseEstimationCount++;

        //if it calculates that were 10 meter away for more than 10 times in a row its probably right
        if (longDistangePoseEstimationCount < 10)
        {
          return Optional.empty();
        }
      } else
      {
        longDistangePoseEstimationCount = 0;
      }
      return pose;
    }
    return Optional.empty();
  }

  /**
   * Get the latest result from a given Camera.
   *
   * @param camera Given camera to take the result from.
   * @return Photon result from sim or a real camera.
   */
  public PhotonPipelineResult getLatestResult(Cameras camera)
  {
    PhotonCamera    cam;
    PhotonCameraSim simCam;
    switch (camera)
    {
      case LEFT_CAM:
      {
        cam = leftCam;
        simCam = simLeftCam;
        break;
      }
      case RIGHT_CAM:
      {
        cam = rightCam;
        simCam = simRightCam;
        break;
      }
      case CENTER_CAM:
      {
        cam = centerCam;
        simCam = simCenterCam;
        break;
      }
      default:
      {
        return null;
      }
    }

    return Robot.isReal() ? cam.getLatestResult() : simCam.getCamera().getLatestResult();
  }

  /**
   * Check if the latest result of a given Camera has any April Tags in view.
   *
   * @param camera Camera to check.
   * @return If the given Camera has any April Tags
   */
  public boolean hasTargets(Cameras camera)
  {
    return getLatestResult(camera).hasTargets();
  }

  /**
   * Get the distance from the pose to the current pose.
   *
   * @param pose Pose to check.
   * @return Distance to the pose.
   */
  public double getDistanceFromPose(Pose2d pose)
  {
    return PhotonUtils.getDistanceToPose(currentPose.get(), pose);
  }

  /**
   * Get AprilTag pose.
   *
   * @param id AprilTagID
   * @return Pose of Tag.
   */
  public Pose2d getTagPose(int id)
  {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    if (tag.isPresent())
    {
      return tag.get().toPose2d();
    }
    return null;
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id)
  {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    if (tag.isPresent())
    {
      return getDistanceFromPose(tag.get().toPose2d());
    }
    return -1;
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera)
  {
    PhotonTrackedTarget  target = null;
    PhotonPipelineResult result = getLatestResult(camera);
    if (result.hasTargets())
    {
      for (PhotonTrackedTarget i : result.getTargets())
      {
        if (i.getFiducialId() == id)
        {
          target = i;
        }
      }
    }
    return target;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim()
  {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on localhost.
   */
  private void openSimCameraViews()
  {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE))
    {
      try
      {
        Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
        Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
        Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      } catch (IOException | URISyntaxException e)
      {
        e.printStackTrace();
      }
    }
  }

  /**
   * Update the field2d to include tracked targets/
   */
  public void updateVisionField()
  {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    if (hasTargets(Cameras.LEFT_CAM))
    {
      targets.addAll(getLatestResult(Cameras.LEFT_CAM).targets);
    }
    if (hasTargets(Cameras.RIGHT_CAM))
    {
      targets.addAll(getLatestResult(Cameras.RIGHT_CAM).targets);
    }
    if (hasTargets(Cameras.CENTER_CAM))
    {
      targets.addAll(getLatestResult(Cameras.CENTER_CAM).targets);
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets)
    {
      Pose2d targetPose = getTagPose(target.getFiducialId());
      poses.add(targetPose);
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to select each camera
   */
  enum Cameras
  {
    /**
     * Left Camera
     */
    LEFT_CAM,
    /**
     * Right Camera
     */
    RIGHT_CAM,
    /**
     * Center Camera
     */
    CENTER_CAM
  }

}

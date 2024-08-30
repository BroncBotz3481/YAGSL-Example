package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
   * Camera Enum to select each camera
   */
  enum Cameras
  {
    /**
     * Left Camera
     */
    LEFT_CAM("left",
             new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
             new Translation3d(Units.inchesToMeters(12.056),
                               Units.inchesToMeters(10.981),
                               Units.inchesToMeters(8.44))),
    /**
     * Right Camera
     */
    RIGHT_CAM("right",
              new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
              new Translation3d(Units.inchesToMeters(12.056),
                                Units.inchesToMeters(-10.981),
                                Units.inchesToMeters(8.44))),
    /**
     * Center Camera
     */
    CENTER_CAM("center",
               new Rotation3d(0, Units.degreesToRadians(18), 0),
               new Translation3d(Units.inchesToMeters(-4.628),
                                 Units.inchesToMeters(-10.687),
                                 Units.inchesToMeters(16.129)));

    /**
     * Transform of the camera rotation and translation relative to the center of the robot
     */
    private final Transform3d robotToCamTransform;

    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;

    /**
     * Camera instance for comms.
     */
    public final PhotonCamera        camera;
    /**
     * Simulated camera instance which only exists during simulations.
     */
    public       PhotonCameraSim     cameraSim;
    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;


    /**
     * Construct a Photon Camera class with help.
     *
     * @param name                  Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation)
    {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.WARNING);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
                                              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                              robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      if (Robot.isSimulation())
      {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim)
    {
      if (Robot.isSimulation())
      {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }
  }

  /**
   * April Tag Field Layout of the year.
   */
  private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);


  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;
  /**
   * Photon Vision camera properties simulation.
   */

  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d          field2d;

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

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values())
      {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }


  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive)
  {
    ArrayList<EstimatedRobotPose> estimatedRobotPoses = getEstimatedGlobalPose();
    if(Robot.isReal()) {
      for (EstimatedRobotPose i : estimatedRobotPoses)
      {
        swerveDrive.addVisionMeasurement(i.estimatedPose.toPose2d(), i.timestampSeconds);
      }
    } else visionSim.update(swerveDrive.getPose());
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
    ArrayList<EstimatedRobotPose> poses = new ArrayList<>();

    for (Cameras c : Cameras.values())
    {
      Optional<EstimatedRobotPose> poseEst = filterPose(c.poseEstimator.update());

      if (poseEst.isPresent())
      {
        poses.add(poseEst.get());
        field2d.getObject(c + " est pose").setPose(poseEst.get().estimatedPose.toPose2d());
      }
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
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1)
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

    return Robot.isReal() ? camera.camera.getLatestResult() : camera.cameraSim.getCamera().getLatestResult();
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
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
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
    for (Cameras c : Cameras.values())
    {
      if (getLatestResult(c).hasTargets())
      {
        targets.addAll(getLatestResult(c).targets);
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets)
    {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent())
      {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag The ID of the AprilTag.
   * @param xOffset The X offset in meters from the AprilTag's position, positive is away from the AprilTag.
   * @param yOffset The Y offset in meters from the AprilTag's position, positive is to the right of the AprilTag
   *                regardless of alliance.
   * @param rotOffset The rotation offset in degrees from the AprilTag's orientation.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Double xOffset, Double yOffset, Double rotOffset)
  {  
    Optional<Pose3d> aprilTagPose3d =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo).getTagPose(aprilTag);

    Pose2d aprilTagPose2d = aprilTagPose3d.get().toPose2d();

    Transform2d aprilTagGoalTrans2d = new Transform2d(new Translation2d(xOffset, yOffset),
                                                      new Rotation2d(Math.toRadians(rotOffset)));

    Pose2d aprilTagTargetPose2d = aprilTagPose2d.transformBy(aprilTagGoalTrans2d);

    return aprilTagTargetPose2d;
  }

}

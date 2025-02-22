package frc.robot.subsystems;

import java.util.Optional;

import limelight.Limelight;
import limelight.networktables.LimelightSettings;
import limelight.networktables.LimelightSettings.LEDMode;

// april tags imports
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.AprilTagFiducial;

public class LimelightSubsystem {
    Limelight limelight;
    LimelightSettings limelightSettings;
    LimelightPoseEstimator poseEstimator;

    public LimelightSubsystem() {
        limelight = new Limelight("limelight-butler");
        limelightSettings = new LimelightSettings(limelight);
        poseEstimator = new LimelightPoseEstimator(limelight, true);
    }

    public void turnOn() { //a
        limelightSettings.withLimelightLEDMode(LEDMode.ForceOn);
    }

    public void turnOff() { //b
        limelightSettings.withLimelightLEDMode(LEDMode.ForceOff);
    }

    public void scanAprilTag() { //x
        Optional<PoseEstimate> poseEstimateResults = poseEstimator.getPoseEstimate();

        if (poseEstimateResults.isPresent()) {
            if (poseEstimateResults.get().hasData) {
                PoseEstimate poseEstimate = poseEstimateResults.get();

                System.out.println(poseEstimate.rawFiducials);
            } else {
                System.out.println("the pose estimate has no data :(");
            }
        } else {
            System.out.println("there is no apriltag :(");
        }
    }
}

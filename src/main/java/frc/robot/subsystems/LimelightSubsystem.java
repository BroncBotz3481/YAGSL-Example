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
        Optional<PoseEstimate> poseEstimate = poseEstimator.getPoseEstimate();

        if (poseEstimate.isPresent()) {
            System.out.println(poseEstimate.get());
        } else {
            System.out.println("there is no apriltag :(");
        }
    }
}

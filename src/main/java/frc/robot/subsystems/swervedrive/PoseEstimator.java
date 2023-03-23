package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import swervelib.SwerveDrive;

public class PoseEstimator extends SubsystemBase {

    SwerveDrive swerveDrive;
    LimelightResults previousResult = null;
    boolean isRedAlliance;

    public PoseEstimator(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        
    }

    public void setCurrentTeamColor() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            System.out.println("Blue Team Configured");
            isRedAlliance = false;
        } else {
            System.out.println("Red Team Configured");
            isRedAlliance = true;
        }
    }

    @Override
    public void periodic() {
        LimelightResults currentResult = LimelightHelpers.getLatestResults("");
        if (currentResult != previousResult) {
            previousResult = currentResult;
            if (isRedAlliance) {
                swerveDrive.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiRed(""), LimelightHelpers.getLatency_Capture(""), true, .5);
            } else {
                swerveDrive.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(""), LimelightHelpers.getLatency_Capture(""), true, .5);
            }
        }
    }
}
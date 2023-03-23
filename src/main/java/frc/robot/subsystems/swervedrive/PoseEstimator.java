// package frc.robot.subsystems.swervedrive;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.LimelightResults;
// import swervelib.SwerveDrive;

// public class PoseEstimator extends SubsystemBase {

//     SwerveSubsystem swerveDrive;
//     LimelightResults previousResult = null;
//     double previousTime = 0;
//     boolean isRedAlliance;

//     public PoseEstimator(SwerveSubsystem swerveDrive) {
//         this.swerveDrive = swerveDrive;
        
//     }

//     public void setCurrentTeamColor() {
//         if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
//             System.out.println("Blue Team Configured");
//             isRedAlliance = false;
//         } else {
//             System.out.println("Red Team Configured");
//             isRedAlliance = true;
//         }
//     }

//     @Override
//     public void periodic() {
//         LimelightResults currentResult = LimelightHelpers.getLatestResults("");
//         double currentTime = Timer.getFPGATimestamp();
        
//         if ((currentTime - previousTime > 10) && (currentResult != previousResult)) {
//             System.out.println("Ten Seconds has passed");
//             previousResult = currentResult;
//             if (isRedAlliance) {
//                 swerveDrive.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiRed(""), LimelightHelpers.getLatency_Capture(""), true, .5);
//                 System.out.println("Using vision pose");
//             } else {
//                 swerveDrive.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(""), LimelightHelpers.getLatency_Capture(""), true, .5);
//                 System.out.println("Using vision pose");
//             }
//         }
//     }
// }
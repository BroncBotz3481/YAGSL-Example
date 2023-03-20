package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

public class PathBuilder {
  private SwerveAutoBuilder autoBuilder;

  public PathBuilder(SwerveSubsystem drivebase, HashMap<String, Command> eventMap) {

    autoBuilder =
        new SwerveAutoBuilder(
            drivebase::getPose, // Functional interface to feed supplier
            drivebase::resetOdometry,
            // Position controllers
            new PIDConstants(Auton.xAutoPID.p, Auton.xAutoPID.i, Auton.xAutoPID.d),
            new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
            drivebase::setChassisSpeeds,
            eventMap,
            false,
            drivebase);
  }

  public Command getSwerveCommand(List<PathPlannerTrajectory> path) {
    System.out.println("COMMAND GET");
    return autoBuilder.fullAuto(path);
  }
}
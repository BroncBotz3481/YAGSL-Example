package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.swerve.commands.CustomSwerveControllerCommand;
import java.util.List;


public class Auto1Command extends CommandBase
{

  private final SwerveSubsystem               swerveSubsystem;
  private final CustomSwerveControllerCommand swerveControllerCommand;
  private final Trajectory                    trajectory;

  public Auto1Command(SwerveSubsystem swerveSubsystem)
  {
    this.swerveSubsystem = swerveSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);

    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                             Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(
            swerveSubsystem.m_drive.m_swerveKinematics);

    // 2. Generate trajectory ---Is all the Poses put together in Autonomous
    trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                        List.of(new Translation2d(1, 0),
                                                                new Translation2d(1, 1)),
                                                        new Pose2d(2, 1, Rotation2d.fromDegrees(180)),
                                                        trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0,
                                                                      new TrapezoidProfile.Constraints(6.28, 3.14));
    thetaController.enableContinuousInput(-180, 180);

    // 4. Construct command to follow trajectory
    // Changed to use the wpilib SwerveControllerCommand
    // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SwerveControllerCommand.html
    swerveControllerCommand = new CustomSwerveControllerCommand(trajectory,
                                                                swerveSubsystem.m_drive::getPose,
                                                                swerveSubsystem.m_drive.m_swerveKinematics,
                                                                xController,
                                                                yController,
                                                                thetaController,
                                                                swerveSubsystem.m_drive::set,
                                                                swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    swerveControllerCommand.initialize();
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    swerveControllerCommand.execute();
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return swerveControllerCommand.isFinished();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    swerveControllerCommand.end(interrupted);
  }

  /**
   * Get the initial position.
   *
   * @return Initial pose.
   */
  public Pose2d getInitialPosition()
  {
    return trajectory.getInitialPose();
  }
}

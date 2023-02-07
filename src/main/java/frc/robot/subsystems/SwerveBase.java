// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.BetterSwerveKinematics;
import frc.lib.util.BetterSwerveModuleState;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Constants.Drivebase;

public class SwerveBase extends SubsystemBase {

  private SwerveModule[] swerveModules;
  private PigeonIMU imu;
  
  private SwerveDriveOdometry odometry;
  public Field2d field = new Field2d();

  private double angle, lasttime;

  private Timer timer;

  private boolean wasGyroReset;

  /** Creates a new swerve drivebase subsystem.  Robot is controlled via the drive() method,
   * or via the setModuleStates() method.  The drive() method incorporates kinematics— it takes a 
   * translation and rotation, as well as parameters for field-centric and closed-loop velocity control.
   * setModuleStates() takes a list of SwerveModuleStates and directly passes them to the modules.
   * This subsytem also handles odometry.
  */
  public SwerveBase() {

    // Create an integrator for angle if the robot is being simulated to emulate an IMU
    // If the robot is real, instantiate the IMU instead.
    if (!Robot.isReal()) {
      timer = new Timer();
      timer.start();
      lasttime = 0;
    } else {
      imu = new PigeonIMU(Drivebase.PIGEON);
      imu.configFactoryDefault();
    }

    this.swerveModules = new SwerveModule[] {
      new SwerveModule(0, Drivebase.Mod0.CONSTANTS),
      new SwerveModule(1, Drivebase.Mod1.CONSTANTS),
      new SwerveModule(2, Drivebase.Mod2.CONSTANTS),
      new SwerveModule(3, Drivebase.Mod3.CONSTANTS),
    };

    odometry = new SwerveDriveOdometry(Drivebase.KINEMATICS, getYaw(), getModulePositions());
    zeroGyro();
  }

  /**
   * The primary method for controlling the drivebase.  Takes a Translation2d and a rotation rate, and 
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity
   * control for the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation
   *  vector is used.
   * @param translation  Translation2d that is the commanded linear velocity of the robot, in meters per second.
   * In robot-relative mode, positive x is torwards the bow (front) and positive y is torwards port (left).  In field-
   * relative mode, positive x is away from the alliance wall (field North) and positive y is torwards the left wall when looking 
   * through the driver station glass (field West).
   * @param rotation  Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot relativity.
   * @param fieldRelative  Drive mode.  True for field-relative, false for robot-relative.
   * @param isOpenLoop  Whether or not to use closed-loop velocity control.  Set to true to disable closed-loop.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if necessary.
    ChassisSpeeds velocity = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      translation.getX(), 
      translation.getY(), 
      rotation, 
      getYaw()
    )
    : new ChassisSpeeds(
      translation.getX(),
      translation.getY(),
      rotation
    );

    // Display commanded speed for testing
    SmartDashboard.putString("RobotVelocity", velocity.toString());

    // Calculate required module states via kinematics
    BetterSwerveModuleState[] swerveModuleStates = 
      Drivebase.KINEMATICS.toSwerveModuleStates(
        velocity
      );
    // Desaturate calculated speeds
    BetterSwerveKinematics.desaturateWheelSpeeds(swerveModuleStates, Drivebase.MAX_SPEED);

    // Command and display desired states
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
      SmartDashboard.putNumber("Module " + module.moduleNumber + " Speed Setpoint: ", swerveModuleStates[module.moduleNumber].speedMetersPerSecond);
      SmartDashboard.putNumber("Module " + module.moduleNumber + " Angle Setpoint: ", swerveModuleStates[module.moduleNumber].angle.getDegrees());
    }
  }

  /**
   * Set the module states (azimuth and velocity) directly.  Used primarily for auto
   * pathing.
   * @param desiredStates  A list of SwerveModuleStates to send to the modules.
   */
  public void setModuleStates(BetterSwerveModuleState[] desiredStates) {
    // Desaturates wheel speeds
    BetterSwerveKinematics.desaturateWheelSpeeds(desiredStates, Drivebase.MAX_SPEED);

    // Sets states
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  /**
   * Set field-relative chassis speeds with closed-loop velocity control.
   * @param chassisSpeeds Field-relative.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
      Drivebase.KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getYaw())));
  }

  
  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(Drivebase.KINEMATICS.toChassisSpeeds(getStates()), getYaw().unaryMinus());
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return Drivebase.KINEMATICS.toChassisSpeeds(getStates());
  }

  
  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to 
   * be reset when calling this method.  However, if either gyro angle or module position
   * is reset, this must be called in order for odometry to keep working.
   * @param pose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /**
   * Gets the current module states (azimuth and velocity)
   * @return A list of SwerveModuleStates containing the current module states
   */
  public BetterSwerveModuleState[] getStates() {
    BetterSwerveModuleState[] states = new BetterSwerveModuleState[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters))
   * @return A list of SwerveModulePositions containg the current module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  /**
   * A public method to allow other systems to determine if the gyro was reset by accessing
   * the wasGyroReset flag.
   * @return The boolean value of wasGyroReset
   */
  public boolean wasGyroReset() {
    return wasGyroReset;
  }

  /**
   * Sets wasGyroReset to false.  Should be called after all systems that need to know have called
   * wasGyroReset.
   */
  public void clearGyroReset() {
    wasGyroReset = false;
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   * Also sets the wasGyroReset flag to true.
   */
  public void zeroGyro() {
    // Resets the real gyro or the angle accumulator, depending on whether the robot is being simulated
    if (Robot.isReal()) {
      imu.setYaw(0);
    } else {
      angle = 0;
    }
    wasGyroReset = true;
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   * @return The yaw angle
   */
  public Rotation2d getYaw() {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    if (Robot.isReal()) {
      double[] ypr = new double[3];
      imu.getYawPitchRoll(ypr);
      return (Drivebase.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    } else {
      return new Rotation2d(angle);
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setMotorBrake(brake);
    }
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move.
   */
  public void setDriveBrake() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(
        new BetterSwerveModuleState(
          0,
          Drivebase.MODULE_LOCATIONS[swerveModule.moduleNumber].getAngle(),
          0),
        true);
    }
  }

  @Override
  public void periodic() {
    // Update odometry
    odometry.update(getYaw(), getModulePositions());

    // Update angle accumulator if the robot is simulated
    if (!Robot.isReal()) {
      angle += Drivebase.KINEMATICS.toChassisSpeeds(getStates()).omegaRadiansPerSecond * (timer.get() - lasttime);
      lasttime = timer.get();
    }

    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("Field", field);

    double[] moduleStates = new double[8];
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber("Module" + module.moduleNumber + "CANCoder", module.getCANCoder());
      SmartDashboard.putNumber("Module" + module.moduleNumber + "Relative Encoder", module.getRelativeEncoder());
      moduleStates[module.moduleNumber] = module.getState().angle.getDegrees();
      moduleStates[module.moduleNumber + 1] = module.getState().speedMetersPerSecond;
    }
    SmartDashboard.putNumberArray("moduleStates", moduleStates);
  }

  @Override
  public void simulationPeriodic() {
  }
}

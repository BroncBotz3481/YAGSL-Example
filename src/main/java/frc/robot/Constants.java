// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class ShooterConstants {
    public static final int SHOOTER_TOP_MOTOR = 20;
    public static final int SHOOTER_BOTTOM_MOTOR = 21;
    public static final int ARM_MAIN_MOTOR = 22;
    public static final int ARM_FOLLOWER_MOTOR = 23;

    public static final double highGoalVelocityTopMotor = .55;
    public static final double highGoalVelocityBottomMotor = .55;

    public static final double midGoalVelocityBottomMotor = .21;
    public static final double midGoalVelocityTopMotor = .22;

    public static final double bottomGoalVelocityTopMotor = .14;
    public static final double bottomGoalVelocityBottomMotor = .13;
    

    public static final int kTimeoutMs = 0;
    public static final int kPIDLoopIdx = 0;
    public static final double intakeVelocity = .28 ;

    /* Sysid values divided by 12 to convert from voltage */
    public static final double armkG = (0 / 12);
    public static final double armkS = (0 / 12);


    public static final double armkP = 0;
    public static final double armkI = 0;
    public static final double armkD = 0;
    public static final double armkF = 0;

    public static final double shooterArmPeakCurrentDuration = 0.1;
    public static final double shooterArmPeakCurrentLimit = 45;
    public static final double shooterArmContinuousCurrentLimit = 30;
    public static final boolean shooterArmEnableCurrentLimit = true;
    public static final double openLoopRamp = 0.25;
    public static final double motionCruiseVelocity = 15000;
    public static final double motionAcceleration = 1500;


    /* Arm Constants */

    public static final double kArmReduction = 16;
    public static final double kArmLengthMeters = 10;
    public static final double kArmMassKg = 10;
    public static final double kMinAngleRads = -2 * Math.PI;
    public static final double kMaxAngleRads = 2 * Math.PI;
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 2048;
  
  }
  public static final class Auton
  {  
    /* Balance PID Values */
    public static final PIDController balancePID = new PIDController(.048, 0.0001, 0.01);

    /* Pathplanner */
    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;

    public static final Translation3d cameraTranslation = new Translation3d(0, 0.0, 0);
    public static final Rotation3d cameraRotation = new Rotation3d(0, 0, 0);
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
  }


}

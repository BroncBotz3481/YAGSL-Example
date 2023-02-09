// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swervedrive2.swervelib.encoders.CANCoderSwerve;
import frc.robot.subsystems.swervedrive2.swervelib.motors.SparkMaxSwerve;
import frc.robot.subsystems.swervedrive2.swervelib.parser.PIDFConfig;
import frc.robot.subsystems.swervedrive2.swervelib.parser.SwerveModuleConfiguration;
import frc.robot.subsystems.swervedrive2.swervelib.parser.SwerveModulePhysicalCharacteristics;

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

  public static final double        ROBOT_MASS       = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double        MANIPULATOR_MASS = 10 * 0.453592; // 10lbs * kg per pound
  public static final double        CHASSIS_MASS     = ROBOT_MASS - MANIPULATOR_MASS;
  public static final Translation3d CHASSIS_CG       = new Translation3d(0, 0, Units.inchesToMeters(8));
  public static final double        ARM_Y_POS        = 0; // centered on robot
  public static final double        GRAVITY          = 9.81; // m/s/s
  public static final double        LOOP_TIME        = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final double dummyArmX      = Units.inchesToMeters(42);
  public static final double dummyArmHieght = Units.inchesToMeters(27);

  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    // Robot heading control gains
    public static final double HEADING_KP = 0.4;
    public static final double HEADING_KI = 0;
    public static final double HEADING_KD = 0.01;

    // Motor and encoder inversions
    public static final int                                 PIGEON                          = 13;
    public static       SwerveModulePhysicalCharacteristics MODULE_PHYSICAL_CHARACTERISTICS =
        new SwerveModulePhysicalCharacteristics(
            6.75,
            12.8,
            5676,
            Units.inchesToMeters(4), .25, .25);

    /**
     * Module locations, in meters, as distances to the center of the robot. Positive x is torwards the robot front, and
     * +y is torwards robot left.
     */
    public static class ModuleLocations
    {

      public static final double FRONT_LEFT_X  = Units.inchesToMeters(11);
      public static final double BACK_LEFT_Y   = Units.inchesToMeters(11);
      public static final double FRONT_LEFT_Y  = Units.inchesToMeters(11);
      public static final double FRONT_RIGHT_X = Units.inchesToMeters(11);
      public static final double FRONT_RIGHT_Y = Units.inchesToMeters(-11);
      public static final double BACK_LEFT_X   = Units.inchesToMeters(-11);
      public static final double BACK_RIGHT_X  = Units.inchesToMeters(-11);
      public static final double BACK_RIGHT_Y  = Units.inchesToMeters(-11);
    }

    public static class DrivetrainLimitations
    {

      // Drivetrain limitations
      /**
       * meters per second
       */
      public static final double MAX_SPEED                = Units.feetToMeters(14.5);
      /**
       * rad/s
       */
      public static final double MAX_ANGULAR_VELOCITY     = MAX_SPEED / Math.hypot(ModuleLocations.FRONT_LEFT_X,
                                                                                   ModuleLocations.FRONT_LEFT_Y);
      /**
       * Theoretical max acceleration should be as follows:
       * <p> (NEO stall torque * module gearing * number of modules) / (wheel radius * robot mass) = m/s/s </p>
       * <p> (2.6 * 6.75 * 4) / (Units.inchesToMeters(2) * ROBOT_MASS) </p>
       * However, the drive is traction-limited, so the max acceleration is actually
       * <p>(wheel coefficient of friction * gravity)</p>
       */
      public static final double MAX_ACCELERATION         = 1.19 * 9.81;
      // COF (blue nitrile on carpet) as reported by Studica
      // https://studica.ca/en/blue-nitrile-roughtop-tread-1-in-wide-10-ft-long
      /**
       * max speed (RPM) / gear ratio, convert to deg/min, divide by 60 for deg/s
       */
      public static final double MAX_MODULE_ANGULAR_SPEED = 360 * (5676 / 12.8) / 60; // deg/s
    }
    // degrees per rotation / gear ratio between module and motor

    public static class ModulePIDFGains
    {

      // Module PIDF gains
      public static final PIDFConfig anglePIDF    = new PIDFConfig(0.01,
                                                                   0,
                                                                   0,
                                                                   0,
                                                                   0);
      public static final PIDFConfig velocityPIDF = new PIDFConfig(0.0020645, // kP from sysid eventually.
                                                                   0,
                                                                   0,
                                                                   0,
                                                                   0);
    }

    // Module specific constants
    public static final class Mod0FL
    { //Front Left

      public static final SwerveModuleConfiguration CONSTANTS = new SwerveModuleConfiguration(
          new SparkMaxSwerve(4, true),
          new SparkMaxSwerve(3, false),
          new CANCoderSwerve(9),
          -114.609,
          ModuleLocations.FRONT_LEFT_X,
          ModuleLocations.FRONT_LEFT_Y,
          ModulePIDFGains.anglePIDF,
          ModulePIDFGains.velocityPIDF,
          DrivetrainLimitations.MAX_SPEED,
          MODULE_PHYSICAL_CHARACTERISTICS);
    }

    public static final class Mod1FR
    { //Front Right

      public static final SwerveModuleConfiguration CONSTANTS = new SwerveModuleConfiguration(
          new SparkMaxSwerve(2, true),
          new SparkMaxSwerve(1, false),
          new CANCoderSwerve(10),
          -50.977,
          ModuleLocations.FRONT_RIGHT_X,
          ModuleLocations.FRONT_RIGHT_Y,
          ModulePIDFGains.anglePIDF,
          ModulePIDFGains.velocityPIDF,
          DrivetrainLimitations.MAX_SPEED,
          MODULE_PHYSICAL_CHARACTERISTICS);
    }

    public static final class Mod2BL
    { //Back Left

      public static final SwerveModuleConfiguration CONSTANTS = new SwerveModuleConfiguration(
          new SparkMaxSwerve(7, true),
          new SparkMaxSwerve(8, false),
          new CANCoderSwerve(12),
          6.504,
          ModuleLocations.BACK_LEFT_X,
          ModuleLocations.BACK_LEFT_Y,
          ModulePIDFGains.anglePIDF,
          ModulePIDFGains.velocityPIDF,
          DrivetrainLimitations.MAX_SPEED,
          MODULE_PHYSICAL_CHARACTERISTICS);
    }

    public static final class Mod3BR
    { //Back Right

      public static final SwerveModuleConfiguration CONSTANTS = new SwerveModuleConfiguration(
          new SparkMaxSwerve(5, true),
          new SparkMaxSwerve(6, false),
          new CANCoderSwerve(11),
          -18.281,
          ModuleLocations.BACK_RIGHT_X,
          ModuleLocations.BACK_RIGHT_Y,
          ModulePIDFGains.anglePIDF,
          ModulePIDFGains.velocityPIDF,
          DrivetrainLimitations.MAX_SPEED,
          MODULE_PHYSICAL_CHARACTERISTICS);
    }
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
  }
}

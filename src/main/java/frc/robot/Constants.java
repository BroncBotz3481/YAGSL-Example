/****************************** Header ******************************\
 Class Name: Constants [final]
 File Name: Constants.java
 Summary: Contains constant subclasses and variables for commands, subsystems, and utility methods
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Dylan Watson
 Email: dylantrwatson@gmail.com
 \********************************************************************/

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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

  public static final class ModuleConstants
  {

    public static final double kWheelDiameterMeters         = Units.inchesToMeters(
        4); //Wheel Diameter is 4 inches - converted to meters
    public static final double kDriveMotorGearRatio         = 6.75; //Gear Ratio
    public static final double kTurningMotorGearRatio       = 12.8; //Gear Ratio
    public static final double kDriveEncoderRot2Meter       = kDriveMotorGearRatio * Math.PI *
                                                              kWheelDiameterMeters; //Takes the Gear Ratio of the
    // multiplies it with 2PI to give you Radians
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter /
                                                              60; //Divides the Drive Motor rotations in Meters by 60
    // Driver motor and multiplies it by PI and the Diameter to get you meters
    public static final double kTurningEncoderRot2Rad       = kTurningMotorGearRatio * 2 *
                                                              Math.PI; //Takes the Turning Motor Gear Ratio and
    // to give you Per Second instead of Per Min
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad /
                                                              60; //Divides the Turning Motor rotations in Radians by
    // 60 to give you Per Second instead of Per Min
    public static final double kPTurning                    = 0.5; //PID Constant P for the Turning Motor. The
    // Proportional term already does a good enough job to tune the Motor so the "D" and "I" are not needed
  }

  public static final class DriveConstants
  {

    public static final double                kTrackWidth      = Units.inchesToMeters(23.75);
    // Distance between right and left wheels
    public static final double                kWheelBase       = Units.inchesToMeters(23.75);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort  = 4;
    public static final int kBackLeftDriveMotorPort   = 7;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kBackRightDriveMotorPort  = 5;

    public static final int kFrontLeftTurningMotorPort  = 3;
    public static final int kBackLeftTurningMotorPort   = 8;
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kBackRightTurningMotorPort  = 6;
    public static final int PigeonCANID                 = 13; //updated

    public static final boolean kFrontLeftTurningEncoderReversed  = true;
    public static final boolean kBackLeftTurningEncoderReversed   = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed  = true;

    public static final boolean kFrontLeftDriveEncoderReversed  = true;
    public static final boolean kBackLeftDriveEncoderReversed   = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed  = false;

    public static final int kFrontLeftAbsoluteEncoderPort  = 9; //updated
    public static final int kBackLeftAbsoluteEncoderPort   = 12; //updated
    public static final int kFrontRightAbsoluteEncoderPort = 10; //updated
    public static final int kBackRightAbsoluteEncoderPort  = 11; //updated

    public static final boolean kFrontLeftAbsoluteEncoderReversed  = false;
    public static final boolean kBackLeftAbsoluteEncoderReversed   = false;
    public static final boolean kFrontRightAbsoluteEncoderReversed = false;
    public static final boolean kBackRightAbsoluteEncoderReversed  = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffset  = -115.576;   // 
    public static final double kBackLeftDriveAbsoluteEncoderOffset   = 72.070;   //
    public static final double kFrontRightDriveAbsoluteEncoderOffset = -173.320;  // 
    public static final double kBackRightDriveAbsoluteEncoderOffset  = 39.551;   //  

    public static final double kPhysicalMaxSpeedMetersPerSecond         = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond              = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond      = //
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond        = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    public static final double kFreeSpeedRpm                                  = 5676;
  }

  public static final class AutoConstants
  {

    public static final double kMaxSpeedMetersPerSecond                       =
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond               = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared         = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController                                  = 1.5;
    public static final double kPYController                                  = 1.5;
    public static final double kPThetaController                              = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class OIConstants
  {

    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis                  = 1;
    public static final int kDriverXAxis                  = 0;
    public static final int kDriverRotAxis                = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.05;
  }
}
 
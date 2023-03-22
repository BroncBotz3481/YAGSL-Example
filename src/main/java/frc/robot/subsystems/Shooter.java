// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX motorBottom; 
  TalonFX motorTop;
  TalonFX rotateMotor;
  TalonFX rotateMotorFollower;
  ArmFeedforward m_armFF;
  TalonFXConfiguration shooterArmFXConfig;

  DoubleLogEntry armPosition;
  DoubleLogEntry armAngleDegrees;
  DoubleLogEntry armVelocity;
  DoubleLogEntry armMotorCurrent;
  DoubleLogEntry armMotorCurrent2;

  DoubleLogEntry shooterSpeed;
  DoubleLogEntry shooterCurrent;



  /** Creates a new Shooter. */
  public Shooter() {
    motorTop = new TalonFX(Constants.ShooterConstants.SHOOTER_TOP_MOTOR);
    motorBottom = new TalonFX(Constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR);
    rotateMotor = new TalonFX(Constants.ShooterConstants.ARM_MAIN_MOTOR);
    rotateMotorFollower = new TalonFX(Constants.ShooterConstants.ARM_FOLLOWER_MOTOR);
    
    m_armFF = new ArmFeedforward(Constants.ShooterConstants.armkS, Constants.ShooterConstants.armkG, 0);

    /* Config Intake Motors */
    motorTop.setNeutralMode(NeutralMode.Coast);
    motorBottom.setNeutralMode(NeutralMode.Coast);
    

    motorTop.setInverted(TalonFXInvertType.Clockwise);
    motorBottom.setInverted(TalonFXInvertType.Clockwise);

    /* Config Arm Motor */
    /* Shooter Arm Configuration */
    SupplyCurrentLimitConfiguration shooterArmSupplyLimit = new SupplyCurrentLimitConfiguration(
      Constants.ShooterConstants.shooterArmEnableCurrentLimit, 
      Constants.ShooterConstants.shooterArmContinuousCurrentLimit, 
      Constants.ShooterConstants.shooterArmPeakCurrentLimit, 
      Constants.ShooterConstants.shooterArmPeakCurrentDuration);
    shooterArmFXConfig = new TalonFXConfiguration();
    
    shooterArmFXConfig.slot0.kP = Constants.ShooterConstants.armkP;
    shooterArmFXConfig.slot0.kI = Constants.ShooterConstants.armkI;
    shooterArmFXConfig.slot0.kD = Constants.ShooterConstants.armkD;
    shooterArmFXConfig.slot0.kF = Constants.ShooterConstants.armkF;
    shooterArmFXConfig.supplyCurrLimit = shooterArmSupplyLimit;
    shooterArmFXConfig.openloopRamp = Constants.ShooterConstants.openLoopRamp;
    
    shooterArmFXConfig.motionCruiseVelocity = Constants.ShooterConstants.motionCruiseVelocity;
    shooterArmFXConfig.motionAcceleration = Constants.ShooterConstants.motionAcceleration;
  

    rotateMotor.configAllSettings(shooterArmFXConfig);
    rotateMotor.setInverted(TalonFXInvertType.Clockwise);
    rotateMotor.configMotionSCurveStrength(0);
    rotateMotor.setNeutralMode(NeutralMode.Brake);
    
    /* Follow Arm Motor */
    rotateMotorFollower.follow(rotateMotor);
    rotateMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);   
    rotateMotorFollower.setNeutralMode(NeutralMode.Brake);

		/* Zero the sensor once on robot boot up */
		rotateMotor.setSelectedSensorPosition(0, Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kTimeoutMs);

    DataLog log = DataLogManager.getLog();
    
    armPosition = new DoubleLogEntry(log, "/arm/position");
    armAngleDegrees = new DoubleLogEntry(log, "/arm/angle");
    armVelocity = new DoubleLogEntry(log, "/arm/velocity");
    armMotorCurrent = new DoubleLogEntry(log, "/arm/curent-1");
    armMotorCurrent2 = new DoubleLogEntry(log, "/arm/curent-2");

    shooterSpeed = new DoubleLogEntry(log, "/shooter/speed");
    shooterCurrent = new DoubleLogEntry(log, "/shooter/current");


  }

  @Override
  public void periodic() {
      armPosition.append(rotateMotor.getSelectedSensorPosition());
      //armAngleDegrees.append(getAngle());
      armVelocity.append(rotateMotor.getSelectedSensorVelocity());
      armMotorCurrent.append(rotateMotor.getStatorCurrent());
      armMotorCurrent2.append(rotateMotorFollower.getStatorCurrent());
      
      // ShuffleboardTab tab = Shuffleboard.getTab("Arm");
      SmartDashboard.putNumber("Shooter Master Falcon Position", rotateMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Shooter Follower Falcon Position", rotateMotorFollower.getSelectedSensorPosition());
      SmartDashboard.putNumber("Shooter Follower Falcon Voltage", rotateMotor.getStatorCurrent());
      
      //  SmartDashboard.putNumber("kP", kP.getDouble(.5));

      SmartDashboard.putNumber("Current Arm", rotateMotor.getStatorCurrent());
  }

  public Command moveArm(DoubleSupplier percent) {
    return this.run(() -> {
      rotateMotor.set(ControlMode.PercentOutput, percent.getAsDouble());
      rotateMotorFollower.follow(rotateMotor);
    });
  }

  public Command moveArmToPosition(double targetPos) {
    return this.run(() -> {
      double armMotorVerticalOffset = 0;

      // we know vertical offset, treat it as 0, then add 90 degrees = pi/2 rad to get angle from horizontal
      double angle = (2*Math.PI / 2048 / 16) * (rotateMotor.getSelectedSensorPosition() - armMotorVerticalOffset) + (Math.PI / 2.0);

      rotateMotor.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, 
                      m_armFF.calculate(angle, rotateMotor.getSelectedSensorVelocity()));

      rotateMotorFollower.follow(rotateMotor);
      // DataLogManager.log("Current Angle: " + angle);
      System.out.println("Current Angle: " + angle);
    }).until(() -> rotateMotor.isMotionProfileFinished());
  }

  public Command shoot(double speedTop, double speedBottom) {
    return this.runEnd(() -> {
      motorTop.set(ControlMode.PercentOutput, -speedTop);
      motorBottom.set(ControlMode.PercentOutput, -speedBottom);
    },(() -> {
      motorTop.set(ControlMode.PercentOutput, 0);
      motorBottom.set(ControlMode.PercentOutput, 0);
    }));
  }

  public Command intake() {
    // Great Chance this wont work //
    return moveArmToPosition(0).andThen(() -> {motorTop.set(ControlMode.PercentOutput, Constants.ShooterConstants.intakeVelocity);
    motorBottom.set(ControlMode.PercentOutput, Constants.ShooterConstants.intakeVelocity);});
    // return this.runEnd(() -> {
    //     moveArmToPosition(9000);

    // motorTop.set(ControlMode.PercentOutput, Constants.ShooterConstants.intakeVelocity);
    // motorBottom.set(ControlMode.PercentOutput, Constants.ShooterConstants.intakeVelocity);
    // },(() -> {
    //     motorTop.set(ControlMode.PercentOutput, 0);
    //     motorBottom.set(ControlMode.PercentOutput, 0);
    //     moveArmToPosition(0);
    // }));
  } // DO THIS UNTILL THE CURRENT IS HIGH ENOUGH MENING A CUBE HAS BEEN INTAKED 

  // public Command moveArmZero() {
  //   return this.run(() -> moveArmToPosition(-10000));
  // }


  public Command resetArm() {
    return this.runOnce(() -> {
        rotateMotor.setSelectedSensorPosition(0);
        rotateMotorFollower.setSelectedSensorPosition(0);

    });
  }
}

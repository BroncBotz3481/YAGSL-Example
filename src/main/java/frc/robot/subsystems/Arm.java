package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  WPI_TalonFX armMotor;
  WPI_TalonFX armMotorFollower;
  ArmFeedforward m_armFF;
  TalonFXConfiguration ArmFXConfig;

  DoubleLogEntry armPosition;
  DoubleLogEntry armAngleDegrees;
  DoubleLogEntry armVelocity;
  DoubleLogEntry armMotorCurrent;
  DoubleLogEntry armMotorCurrent2;
  DoubleLogEntry armMotorTemp;

  boolean hasArmBeenReset;

  public enum ArmPosition {
    Intake,
    Low,
    Mid,
    High,
  }
  /** Creates a new Shooter. */
  public Arm() {
    hasArmBeenReset = false;
    armMotor = new WPI_TalonFX(Constants.ArmConstants.ARM_MAIN_MOTOR);
    armMotorFollower = new WPI_TalonFX(Constants.ArmConstants.ARM_FOLLOWER_MOTOR);
    
    m_armFF = new ArmFeedforward(Constants.ArmConstants.armkS, Constants.ArmConstants.armkG, 0);

    configMotors();
    startLogging();
  }

  public void configMotors() {
    armMotor.configFactoryDefault();
    armMotorFollower.configFactoryDefault();

    /* Config Arm Motor */
    ArmFXConfig = new TalonFXConfiguration();
    
    ArmFXConfig.slot0.kP = Constants.ArmConstants.armkP;
    ArmFXConfig.slot0.kI = Constants.ArmConstants.armkI;
    ArmFXConfig.slot0.kD = Constants.ArmConstants.armkD;
    ArmFXConfig.slot0.kF = Constants.ArmConstants.armkF;
    ArmFXConfig.openloopRamp = Constants.ArmConstants.openLoopRamp;
    
    ArmFXConfig.motionCruiseVelocity = Constants.ArmConstants.motionCruiseVelocity;
    ArmFXConfig.motionAcceleration = Constants.ArmConstants.motionAcceleration;
  
    armMotor.configVoltageCompSaturation(10);
    armMotor.enableVoltageCompensation(true);
    armMotor.configAllSettings(ArmFXConfig);

    armMotor.setInverted(TalonFXInvertType.Clockwise);
    armMotor.setSensorPhase(true);

    armMotor.configMotionSCurveStrength(0);
    armMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 45, 1));
    armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.ArmConstants.shooterArmContinuousCurrentLimit, Constants.ArmConstants.shooterArmPeakCurrentLimit, Constants.ArmConstants.shooterArmPeakCurrentDuration));
    
    armMotorFollower.configVoltageCompSaturation(10);
    armMotorFollower.enableVoltageCompensation(true);
    armMotorFollower.follow(armMotor);
    
    armMotorFollower.setInverted(TalonFXInvertType.OpposeMaster);
    armMotorFollower.setSensorPhase(false);

    armMotorFollower.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 45, 1));
    armMotorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.ArmConstants.shooterArmContinuousCurrentLimit, Constants.ArmConstants.shooterArmPeakCurrentLimit, Constants.ArmConstants.shooterArmPeakCurrentDuration));
    

    armMotor.setSelectedSensorPosition(0, Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kTimeoutMs);
    armMotorFollower.setSelectedSensorPosition(0, Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kTimeoutMs);
    setBrake(NeutralMode.Brake);
  }

  public void startLogging() {
    DataLog log = DataLogManager.getLog();

    armPosition = new DoubleLogEntry(log, "/arm/position");
    armAngleDegrees = new DoubleLogEntry(log, "/arm/angle");
    armVelocity = new DoubleLogEntry(log, "/arm/velocity");
    armMotorCurrent = new DoubleLogEntry(log, "/arm/curent-main");
    armMotorCurrent2 = new DoubleLogEntry(log, "/arm/curent-follow");
    armMotorTemp = new DoubleLogEntry(log, "/arm/temp");
  }

  @Override
  public void periodic() {
      armPosition.append(armMotor.getSelectedSensorPosition());
      //armAngleDegrees.append(getAngle());
      armVelocity.append(armMotor.getSelectedSensorVelocity());
      armMotorCurrent.append(armMotor.getStatorCurrent());
      armMotorCurrent2.append(armMotorFollower.getStatorCurrent());
      armMotorTemp.append(armMotor.getTemperature());

      SmartDashboard.putBoolean("ArmReset", hasArmBeenReset);
      SmartDashboard.putNumber("ArmSensorPosition", armMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("ArmSensorVelocity", armMotor.getSelectedSensorVelocity());
      SmartDashboard.putNumber("AppliedThrottle", armMotor.getMotorOutputPercent());

      SmartDashboard.putNumber("ArmStatorCurrent", armMotor.getStatorCurrent());
      SmartDashboard.putNumber("ArmSupplyCurrent", armMotor.getSupplyCurrent());
  }

  public Command moveArm(DoubleSupplier percent) {
    return this.run(() -> {
      armMotor.set(ControlMode.PercentOutput, percent.getAsDouble());
      armMotorFollower.follow(armMotor);
    });
  }

  public Command moveArmToPosition(ArmPosition position) {
    
    return this.run(() -> {
      double targetPos;
      targetPos = 0;
      switch (position) {
        case Intake: targetPos = 100;
                      break;
        case High: targetPos = 0;
          break;
        case Low: break;
        case Mid: break;
        default: break;
      }
      double armMotorVerticalOffset = 0;

      // we know vertical offset, treat it as 0, then add 90 degrees = pi/2 rad to get angle from horizontal
      double angle = (2*Math.PI / 2048 / 16) * (armMotor.getSelectedSensorPosition() - armMotorVerticalOffset) + (Math.PI / 2.0);

      armMotor.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, 
                      m_armFF.calculate(angle, armMotor.getSelectedSensorVelocity()));

      armMotorFollower.follow(armMotor);
      // DataLogManager.log("Current Angle: " + angle);
      System.out.println("Current Angle: " + angle);
    }).until(() -> armMotor.isMotionProfileFinished());
  }

  public Command resetArm() {
    System.out.println("Arm Reset");
    hasArmBeenReset = true;
    return this.runOnce(() -> {
        armMotor.setSelectedSensorPosition(0);
        armMotorFollower.setSelectedSensorPosition(0);
    });
  }

  public void setBrake(NeutralMode breakMode) {
    armMotor.setNeutralMode(breakMode);
    armMotorFollower.setNeutralMode(breakMode);
  }
}

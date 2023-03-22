package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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


  /** Creates a new Shooter. */
  public Arm() {
    armMotor = new WPI_TalonFX(Constants.ArmConstants.ARM_MAIN_MOTOR);
    armMotorFollower = new WPI_TalonFX(Constants.ArmConstants.ARM_FOLLOWER_MOTOR);
    
    m_armFF = new ArmFeedforward(Constants.ArmConstants.armkS, Constants.ArmConstants.armkG, 0);

    configMotors();
    startLogging();
  }

  public void configMotors() {
    /* Config Arm Motor */
    /* Shooter Arm Configuration */
    SupplyCurrentLimitConfiguration shooterArmSupplyLimit = new SupplyCurrentLimitConfiguration(
      Constants.ArmConstants.shooterArmEnableCurrentLimit, 
      Constants.ArmConstants.shooterArmContinuousCurrentLimit, 
      Constants.ArmConstants.shooterArmPeakCurrentLimit, 
      Constants.ArmConstants.shooterArmPeakCurrentDuration);
    ArmFXConfig = new TalonFXConfiguration();
    
    ArmFXConfig.slot0.kP = Constants.ArmConstants.armkP;
    ArmFXConfig.slot0.kI = Constants.ArmConstants.armkI;
    ArmFXConfig.slot0.kD = Constants.ArmConstants.armkD;
    ArmFXConfig.slot0.kF = Constants.ArmConstants.armkF;
    ArmFXConfig.supplyCurrLimit = shooterArmSupplyLimit;
    ArmFXConfig.openloopRamp = Constants.ArmConstants.openLoopRamp;
    
    ArmFXConfig.motionCruiseVelocity = Constants.ArmConstants.motionCruiseVelocity;
    ArmFXConfig.motionAcceleration = Constants.ArmConstants.motionAcceleration;
  

    armMotor.configAllSettings(ArmFXConfig);
    armMotor.setInverted(TalonFXInvertType.Clockwise);
    armMotor.configMotionSCurveStrength(0);
    armMotor.setNeutralMode(NeutralMode.Brake);
    
    /* Follow Arm Motor */
    armMotorFollower.follow(armMotor);
    armMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);   
    armMotorFollower.setNeutralMode(NeutralMode.Brake);

		/* Zero the sensor once on robot boot up */
		armMotor.setSelectedSensorPosition(0, Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kTimeoutMs);
  }

  public void startLogging() {
    DataLog log = DataLogManager.getLog();

    armPosition = new DoubleLogEntry(log, "/arm/position");
    armAngleDegrees = new DoubleLogEntry(log, "/arm/angle");
    armVelocity = new DoubleLogEntry(log, "/arm/velocity");
    armMotorCurrent = new DoubleLogEntry(log, "/arm/curent-1");
    armMotorCurrent2 = new DoubleLogEntry(log, "/arm/curent-2");
  }

  @Override
  public void periodic() {
      armPosition.append(armMotor.getSelectedSensorPosition());
      //armAngleDegrees.append(getAngle());
      armVelocity.append(armMotor.getSelectedSensorVelocity());
      armMotorCurrent.append(armMotor.getStatorCurrent());
      armMotorCurrent2.append(armMotorFollower.getStatorCurrent());

      
      SmartDashboard.putNumber("Shooter Master Falcon Position", armMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Shooter Follower Falcon Position", armMotorFollower.getSelectedSensorPosition());
      SmartDashboard.putNumber("Shooter Follower Falcon Voltage", armMotor.getStatorCurrent());

      SmartDashboard.putNumber("Current Arm", armMotor.getStatorCurrent());
  }

  public Command moveArm(DoubleSupplier percent) {
    return this.run(() -> {
      armMotor.set(ControlMode.PercentOutput, percent.getAsDouble());
      armMotorFollower.follow(armMotor);
    });
  }

  public Command moveArmToPosition(double targetPos) {
    return this.run(() -> {
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
    return this.runOnce(() -> {
        armMotor.setSelectedSensorPosition(0);
        armMotorFollower.setSelectedSensorPosition(0);
    });
  }
}

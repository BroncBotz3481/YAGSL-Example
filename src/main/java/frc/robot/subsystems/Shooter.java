package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonFX shootMotor; 
  WPI_TalonFX shootMotorFollower;

  DoubleLogEntry shooterSpeed;
  DoubleLogEntry shooterCurrent;

  public enum ShootSpeed {
    Stop,
    High,
    Mid,
    Low
  } 
  /** Creates a new Shooter. */
  public Shooter() {
    shootMotorFollower = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_TOP_MOTOR);
    shootMotor = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR);
    
    configMotors();
    startLogging();
  }

  private void configMotors() {
    /* Config Intake Motors */
    shootMotor.setInverted(TalonFXInvertType.Clockwise);
    shootMotor.setNeutralMode(NeutralMode.Coast);
    shootMotor.configVoltageCompSaturation(11); // "full output" will now scale to 11 Volts for all control modes when enabled.
    shootMotor.enableVoltageCompensation(true);

    shootMotorFollower.setInverted(TalonFXInvertType.Clockwise);
    shootMotorFollower.setNeutralMode(NeutralMode.Coast);
    shootMotorFollower.configVoltageCompSaturation(11); // "full output" will now scale to 11 Volts for all control modes when enabled.
    shootMotorFollower.enableVoltageCompensation(true);
  }

  private void startLogging() {
    DataLog log = DataLogManager.getLog();

    shooterSpeed = new DoubleLogEntry(log, "/shooter/speed");
    shooterCurrent = new DoubleLogEntry(log, "/shooter/current");
  }

  @Override
  public void periodic() {
  }

  public Command shoot(double speedTop, double speedBottom) {
    return this.runEnd(() -> {
      shootMotorFollower.set(ControlMode.PercentOutput, -speedTop);
      shootMotor.set(ControlMode.PercentOutput, -speedBottom);
    },(() -> {
      shootMotorFollower.set(ControlMode.PercentOutput, 0);
      shootMotor.set(ControlMode.PercentOutput, 0);
    }));
  }

  public Command shoot(ShootSpeed shootSpeed) {
    return this.runEnd(() -> {
      double speedTop;
      double speedBottom;
      switch (shootSpeed) {
        case Low: 
          speedTop = Constants.ShooterConstants.bottomGoalVelocityTopMotor;
          speedBottom = Constants.ShooterConstants.bottomGoalVelocityBottomMotor;
          break;
        case Mid: 
          speedTop = Constants.ShooterConstants.midGoalVelocityTopMotor;
          speedBottom = Constants.ShooterConstants.midGoalVelocityBottomMotor;
          break;
        case High: 
          speedTop = Constants.ShooterConstants.highGoalVelocityTopMotor;
          speedBottom = Constants.ShooterConstants.highGoalVelocityBottomMotor;
          break;
        default: 
          speedTop = 0;
          speedBottom = 0;
          break;
      }
      
      shootMotorFollower.set(ControlMode.PercentOutput, -speedTop);
      shootMotor.set(ControlMode.PercentOutput, -speedBottom);
    },(() -> {
      shootMotorFollower.set(ControlMode.PercentOutput, 0);
      shootMotor.set(ControlMode.PercentOutput, 0);
    }));
  }

  public Command intake() {
    return this.run(() -> {
      shootMotorFollower.set(ControlMode.PercentOutput, Constants.ShooterConstants.intakeVelocity);
      shootMotor.set(ControlMode.PercentOutput, Constants.ShooterConstants.intakeVelocity);
    }).until(() -> shootMotor.getStatorCurrent() > Constants.ShooterConstants.currentThreshold);
  }
}

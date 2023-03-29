package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonFX shootMotor; 
  WPI_TalonFX shootMotorFollower;

  DoubleLogEntry shooterSpeed;
  DoubleLogEntry shooterCurrent;
  double pValue;
  public enum ShootSpeed {
    Stop,
    High,
    Mid,
    Low,
    Intake
  } 
  /** Creates a new Shooter. */
  public Shooter() {
    shootMotorFollower = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_TOP_MOTOR);
    shootMotor = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_BOTTOM_MOTOR);
    pValue = Shuffleboard.getTab("Shoot").addPersistent("PValue", 0).getEntry().getDouble(0);
    configMotors();
    startLogging();
  }

  private void configMotors() {
    /* Config Intake Motors */
    shootMotor.setInverted(TalonFXInvertType.Clockwise);
    shootMotor.setNeutralMode(NeutralMode.Coast);
    shootMotor.configVoltageCompSaturation(10);
    shootMotor.enableVoltageCompensation(true);

    shootMotorFollower.setInverted(TalonFXInvertType.Clockwise);
    shootMotorFollower.setNeutralMode(NeutralMode.Coast);
    shootMotorFollower.configVoltageCompSaturation(10); // "full output" will now scale to 11 Volts for all control modes when enabled.
    shootMotorFollower.enableVoltageCompensation(true);
  }

  private void startLogging() {
    DataLog log = DataLogManager.getLog();

    shooterSpeed = new DoubleLogEntry(log, "/shooter/speed");
    shooterCurrent = new DoubleLogEntry(log, "/shooter/current");
  }

  @Override
  public void periodic() {
    shooterSpeed.append(shootMotor.getSelectedSensorPosition());
    shooterCurrent.append(shootMotor.getStatorCurrent());
    SmartDashboard.putNumber("StatorCurrentShooterTopIntake", shootMotorFollower.getStatorCurrent());
    // System.out.println(Shuffleboard.getTab("Shoot").addPersistent("PValue", 0).getEntry().getDouble(0));
  }

  public Command shoot(double speedTop, double speedBottom) {
    return this.runEnd(() -> {
      shootMotorFollower.set(ControlMode.PercentOutput, speedTop);
      shootMotor.set(ControlMode.PercentOutput, speedBottom);
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
        case Intake: 
          speedTop = -Constants.ShooterConstants.intakeVelocity;
          speedBottom = -Constants.ShooterConstants.intakeVelocity;
        break;
        default: 
          speedTop = 0;
          speedBottom = 0;
          break;
      }
      
      shootMotorFollower.set(ControlMode.PercentOutput, speedTop);
      shootMotor.set(ControlMode.PercentOutput, speedBottom);
    },(() -> {
      shootMotorFollower.set(ControlMode.PercentOutput, 0);
      shootMotor.set(ControlMode.PercentOutput, 0);
    }));
  }

  public Command intake() {
    return this.runEnd(() -> {
      shoot(ShootSpeed.Intake);
    }, (() -> { 
      shoot(ShootSpeed.Stop);  
      }));//.until(() -> shootMotor.getStatorCurrent() > Constants.ShooterConstants.currentThreshold).andThen(shoot(ShootSpeed.Stop));
  }

  public Command intakeCurrent() {
    return this.runEnd(() -> {
      shoot(ShootSpeed.Intake);
    }, (() -> { 
      shoot(ShootSpeed.Stop);     
      })).withTimeout(1).until(() -> shootMotor.getStatorCurrent() > Constants.ShooterConstants.currentThreshold).andThen(shoot(ShootSpeed.Stop));
  }
}

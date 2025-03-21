package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

// for diagnostics
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {
    SparkMax climber = new SparkMax(16, MotorType.kBrushless); // TODO: temporary value, replace later

    double climberSpeed = 0.0;
    
    public void climberIn() {
        setClimberSpeed(-0.15); // TODO: check if this is correct
    }

    public void climberOut() {
        setClimberSpeed(0.15); // TODO: check if this is correct
    }

    public void climberStop() {
        setClimberSpeed(0);
    }

    public void setClimberSpeed(double speed) {
        climberSpeed = speed;
        climber.set(climberSpeed);
    }

    public void putClimberDiagnostics() {
        SmartDashboard.putNumber("Climber Spin Speed", climberSpeed);
        SmartDashboard.putNumber("Climber Spin Speed Percentage", climberSpeed * 100.0);
    }
}

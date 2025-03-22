package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

// for diagnostics
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* NOTE TO SELF:
 * Motor speeds are handled from 0.0 to 0.1,
 * with the speed acting as a percentage
 * of the total speed that the motor can handle.
 * 
 * Positive values spin clockwise.
 * Negative values spin counter-clockwise.
 */

public class IntakeSubsystem extends SubsystemBase {
    SparkMax intake = new SparkMax(23, MotorType.kBrushless); // TODO: temporary value, replace later

    double intakeSpeed = 0.0; // this is here to assist with debugging
    
    public void intakeIn() {
        setIntakeSpeed(-0.1); // TODO: check if this is correct
    }

    public void intakeOut() {
        setIntakeSpeed(0.1); // TODO: check if this is correct
    }

    public void intakeStop() {
        setIntakeSpeed(0);
    }

    private void setIntakeSpeed(double speed) {
        intakeSpeed = speed;
        intake.set(intakeSpeed);
    }

    public void putIntakeDiagnostics() {
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        SmartDashboard.putNumber("Intake Speed Percentage", intakeSpeed * 100.0);
    }
}

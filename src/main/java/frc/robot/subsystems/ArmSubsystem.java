package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

// for diagnostics
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

public class ArmSubsystem extends SubsystemBase {
    SparkMax sparkMax = new SparkMax(9, MotorType.kBrushed);
    Joystick joystick = new Joystick(1);
    double armSpeed = 0.0;

    public void adjustArmSpeed() { // apologies for the duplicate functions but this basically sets the arm speed
        setArmSpeed(joystick.getRawAxis(1)); // this sucks but it will do for now
    }

    private void setArmSpeed(double speed) {
        armSpeed = speed * -0.5;
        sparkMax.set(armSpeed);
    }

    public void stopArm() {
        armSpeed = 0;
        sparkMax.stopMotor();
    }

    public void putArmDiagnostics() { // show arm diagnostics in the smart dashboard
        SmartDashboard.putNumber("Joystick X Raw", joystick.getRawAxis(0));
        SmartDashboard.putNumber("Joystick Y Raw", joystick.getRawAxis(1));
        SmartDashboard.putNumber("Joystick Y Adjusted", joystick.getRawAxis(1) * -1);
        SmartDashboard.putBoolean("Arm Enabled?", joystick.getRawButton(1));
        SmartDashboard.putNumber("Arm Speed Raw", armSpeed);
        SmartDashboard.putNumber("Arm Speed Percentage", armSpeed * 100);
    }
}

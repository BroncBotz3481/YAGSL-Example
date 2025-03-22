package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

// for diagnostics
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorSubsystem extends SubsystemBase {
    SparkMax elevator = new SparkMax(21, MotorType.kBrushless); // TODO: FIX THIS, IS BROKEN!!!!
    DigitalInput topLimitSwitch = new DigitalInput(0); // TODO: temporary values, replace later
    DigitalInput bottomLimitSwitch = new DigitalInput(1);

    Joystick joystick = new Joystick(1);

    double elevatorSpeed = 0.0;


    public void getJoystickInput() {
        setElevatorSpeed(joystick.getRawAxis(1));
    }

    public void setElevatorSpeed(double speed) {
        elevatorSpeed = speed;

        if (speed > 0) {
            if (topLimitSwitch.get()) {
                // We are going up and top limit is tripped so stop
                elevator.set(0);
            } else {
                // We are going up but top limit is not tripped so go at commanded speed
                elevator.set(speed);
            }
        } else {
            if (bottomLimitSwitch.get()) {
                // We are going down and bottom limit is tripped so stop
                elevator.set(0);
            } else {
                // We are going down but bottom limit is not tripped so go at commanded speed
                elevator.set(speed);
            }
        }
    }

    public void stopElevator() {
        setElevatorSpeed(0);
    }

    public void putElevatorDiagnostics() { // show elevator diagnostics in the smart dashboard
        SmartDashboard.putNumber("Joystick X Raw", joystick.getRawAxis(0));
        SmartDashboard.putNumber("Joystick Y Raw", joystick.getRawAxis(1));
        SmartDashboard.putNumber("Joystick Y Adjusted", joystick.getRawAxis(1) * -1);
        SmartDashboard.putBoolean("Elevator Enabled?", joystick.getRawButton(1));
        SmartDashboard.putNumber("Elevator Speed Raw", elevatorSpeed);
        SmartDashboard.putNumber("Elevator Speed Percentage", elevatorSpeed * 100.0);
    }
}

package frc.robot.subsystems.swervedrive.ActuatorSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ActuatorSubsystem extends SubsystemBase {
    private final SparkMax actuator;
    public ActuatorSubsystem(){
        actuator = new SparkMax(5, MotorType.kBrushed);
    }

    public void pull(double x){
        actuator.set(x);
    }

    public void push(double x){
        actuator.set(-x);
    }

    public void stop(){
        actuator.set(0.0);
    }
}

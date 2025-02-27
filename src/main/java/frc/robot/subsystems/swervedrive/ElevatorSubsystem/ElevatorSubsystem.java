package frc.robot.subsystems.swervedrive.ElevatorSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevator;
    public ElevatorSubsystem(){
        elevator = new SparkMax(6, MotorType.kBrushless);
    }

    public void moveUp(double x){
        elevator.set(x);
        System.out.println("encoder:" + elevator.getEncoder().getPosition());
    }

    public void moveDown(double x){
        elevator.set(-x);
        System.out.println("encoder:" + elevator.getEncoder().getPosition())
        ;
    }

    public void stop(){
        elevator.set(0.0);
    }
}

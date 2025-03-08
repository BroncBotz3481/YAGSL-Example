package frc.robot.subsystems.swervedrive.ClawSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private final SparkMax claw;
    public ClawSubsystem(){
        claw = new SparkMax(6, MotorType.kBrushless); // ADD ID!!!!
    }

    public void moveUp(double x){
        claw.set(x);       
    }

    public void moveDown(double x){
        claw.set(-x);
    }

    public double getEncoderValue(){
        return claw.getEncoder().getPosition();
    }

    public void stop(){
        claw.set(0.0);
    }
}

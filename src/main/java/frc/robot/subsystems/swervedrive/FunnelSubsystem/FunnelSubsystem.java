package frc.robot.subsystems.swervedrive.FunnelSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {
    private final SparkMax funnel;
    public FunnelSubsystem(){
        funnel = new SparkMax(7, MotorType.kBrushed);
    }

    public void moveUp(double x){
        funnel.set(x);
        //System.out.println("encoder:" + funnel.getEncoder().getPosition());
    }

    public void moveDown(double x){
        funnel.set(-x);
        //System.out.println("encoder:" + funnel.getEncoder().getPosition());
    }

    public double getEncoderValue(){
        return funnel.getEncoder().getPosition();
    }

    public void stop(){
        funnel.set(0.0);
    }
}

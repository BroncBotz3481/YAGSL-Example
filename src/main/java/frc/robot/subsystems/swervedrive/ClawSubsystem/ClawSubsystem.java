package frc.robot.subsystems.swervedrive.ClawSubsystem;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private final SparkMax claw;
    private final SparkMax kicker;
    public ClawSubsystem(){
        claw = new SparkMax(10, MotorType.kBrushed);
        kicker = new SparkMax(8, MotorType.kBrushed);
    }

    public void move(DoubleSupplier speed){
        claw.set(speed.getAsDouble());       
    }

    public void kickerMove(double speed){
        kicker.set(speed);
    }

    public double getEncoderValue(){
        return claw.getEncoder().getPosition();
    }

    public void stop(){
        claw.set(0.0);
    }

    public void stopKicker(){
        kicker.set(0.0);
    }
}

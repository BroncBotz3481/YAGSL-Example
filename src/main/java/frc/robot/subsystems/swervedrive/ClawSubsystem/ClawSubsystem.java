package frc.robot.subsystems.swervedrive.ClawSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Constants.LimitConstants;;

public class ClawSubsystem extends SubsystemBase {
    private final SparkMax claw;
    private final SparkMax kicker;
    PIDController pid = new PIDController(1, 0, 0);
    public ClawSubsystem(){
        claw = new SparkMax(10, MotorType.kBrushed);
        kicker = new SparkMax(8, MotorType.kBrushed);
    }

    public void move(Double speed){
        if(speed > 0 && getEncoderValue() >= LimitConstants.CLAW_UPPER_LIMIT){
            speed = 0d;
        }
        if(speed < 0 && getEncoderValue() <= LimitConstants.CLAW_LOWER_LIMIT){
            speed = 0d;
        }
        claw.set(speed);       
        
    }

    public void moveTo(double setpoint){
        claw.set(pid.calculate(claw.getEncoder().getPosition(), setpoint));
    }

    public void kickerMove(double speed){
        kicker.set(speed);
    }

    public double getEncoderValue(){
        return claw.getEncoder().getPosition();
    }

    public void stopClaw(){
        claw.set(0.0);
    }

    public void stopKicker(){
        kicker.set(0.0);
    }
}

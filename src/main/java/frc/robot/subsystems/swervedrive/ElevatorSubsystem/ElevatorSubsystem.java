package frc.robot.subsystems.swervedrive.ElevatorSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Constants.LimitConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevator;
    double offset;
    double upperLimit = LimitConstants.ELEVATOR_HEIGHT_LIMIT;
    DigitalInput bottomMagEncoder = new DigitalInput(0);
    public ElevatorSubsystem(){
        elevator = new SparkMax(6, MotorType.kBrushless);
    }

    public void move(double speed){
        if(speed < 0 && !bottomMagEncoder.get()){
            offset = elevator.getEncoder().getPosition();
            speed = 0;
        }
        if(speed > 0 && actualEncoderValue() >= upperLimit){
            speed = 0;
        }

        elevator.set(speed);

    }

    public double getEncoderValue(){
        return elevator.getEncoder().getPosition();
    }

    public void stop(){
        elevator.set(0.0);
    }

    private double actualEncoderValue(){
        return elevator.getEncoder().getPosition() - offset;
    }
}

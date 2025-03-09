package frc.robot.commands.swervedrive.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;

public class MoveElevator extends Command {
    ElevatorSubsystem elevatorSubsystem;
    DoubleSupplier velocity;

    public MoveElevator(ElevatorSubsystem elevatorSubsystem, DoubleSupplier velocity){
        this.elevatorSubsystem = elevatorSubsystem;
        this.velocity = velocity;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute(){
        //if(elevatorSubsystem.getEncoderValue() < 10){
        elevatorSubsystem.move(velocity.getAsDouble());
        //System.out.println(elevatorSubsystem.getEncoderValue());
        //}
        // else{
        //     elevatorSubsystem.stop();
        // }
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
        System.out.println("stoped moving elevator");
    }
}

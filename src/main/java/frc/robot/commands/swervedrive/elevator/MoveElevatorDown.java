package frc.robot.commands.swervedrive.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;

public class MoveElevatorDown extends Command {
    ElevatorSubsystem elevatorSubsystem;
    double speed;

    public MoveElevatorDown(ElevatorSubsystem elevatorSubsystem, double speed){
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute(){
        //if(elevatorSubsystem.getEncoderValue() < 10){
        elevatorSubsystem.moveDown(speed);
        //}
        // else{
        //     elevatorSubsystem.stop();
        // }
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
        System.out.println("stoped moving elevator down");
    }
}

package frc.robot.commands.swervedrive.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;

public class MoveDown extends Command {
    ElevatorSubsystem elevatorSubsystem;
    double speed;

    public MoveDown(ElevatorSubsystem elevatorSubsystem, double speed){
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute(){
        elevatorSubsystem.moveDown(speed);
        System.out.println("moving down");
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
        System.out.println("stoped moving down");
    }
}

package frc.robot.commands.swervedrive.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;

public class MoveUp extends Command {
    ElevatorSubsystem elevatorSubsystem;
    double speed;

    public MoveUp(ElevatorSubsystem elevatorSubsystem, double speed){
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute(){
        elevatorSubsystem.moveUp(speed);
        System.out.println("moving up");
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
        System.out.println("stoped moving up");
    }
}

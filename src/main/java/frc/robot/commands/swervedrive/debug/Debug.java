package frc.robot.commands.swervedrive.debug;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;

public class Debug extends Command {
    ElevatorSubsystem elevatorSubsystem;

    public Debug( ElevatorSubsystem elevatorSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void execute(){
        System.out.println(elevatorSubsystem.getEncoderValue());
    }


}

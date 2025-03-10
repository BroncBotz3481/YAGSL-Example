package frc.robot.commands.swervedrive.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.utilities.Constants.AutoConstants;

public class MoveElevatorHighCoral extends Command {
    ElevatorSubsystem elevatorSubsystem;
    DoubleSupplier velocity;

    public MoveElevatorHighCoral(ElevatorSubsystem elevatorSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute(){

        elevatorSubsystem.moveTo(AutoConstants.HIGH_CORAL_ELEVATOR_POSITION);

    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
        System.out.println("stoped moving elevator to high coral");
    }
}

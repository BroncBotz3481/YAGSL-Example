package frc.robot.commands.swervedrive.debug;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem.ElevatorSubsystem;

public class Debug extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ClawSubsystem clawSubsystem;

    public Debug( ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public void execute(){
        System.out.println("claw:" + clawSubsystem.getEncoderValue());
        System.out.println("elevator:" + elevatorSubsystem.getEncoderValue());

    }


}

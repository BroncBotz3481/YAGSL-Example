package frc.robot.commands.swervedrive.actuator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ActuatorSubsystem.ActuatorSubsystem;

public class Pull extends Command {
    ActuatorSubsystem actuatorSubsystem;
    double speed;

    public Pull(ActuatorSubsystem actuatorSubsystem, double speed){
        this.actuatorSubsystem = actuatorSubsystem;
        this.speed = speed;
        addRequirements(actuatorSubsystem);
    }

    @Override
    public void execute(){
        actuatorSubsystem.pull(speed);
        System.out.println("pulling");
    }

    @Override
    public void end(boolean interrupted){
        actuatorSubsystem.stop();
        System.out.println("stoped pulling");
    }
}

package frc.robot.commands.swervedrive.Actuator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ActuatorSubsystem.ActuatorSubystem;

public class Pull extends Command {
    ActuatorSubystem actuatorSubystem;
    double speed;

    public Pull(ActuatorSubystem actuatorSubystem, double speed){
        this.actuatorSubystem = actuatorSubystem;
        this.speed = speed;
        addRequirements(actuatorSubystem);
    }

    @Override
    public void execute(){
        actuatorSubystem.pull(speed);
        System.out.println("pulling");
    }

    @Override
    public void end(boolean interrupted){
        actuatorSubystem.stop();
        System.out.println("stoped pulling");
    }
}

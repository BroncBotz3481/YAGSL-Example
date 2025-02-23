package frc.robot.commands.swervedrive.Actuator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ActuatorSubsystem.ActuatorSubystem;

public class Push extends Command {
    ActuatorSubystem actuatorSubystem;
    double speed;

    public Push(ActuatorSubystem actuatorSubystem, double speed){
        this.actuatorSubystem = actuatorSubystem;
        this.speed = speed;
        addRequirements(actuatorSubystem);
    }

    @Override
    public void execute(){
        actuatorSubystem.push(speed);
    }

    @Override
    public void end(boolean interrupted){
        actuatorSubystem.stop();
    }
}

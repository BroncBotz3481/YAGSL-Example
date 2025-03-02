package frc.robot.commands.swervedrive.actuator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ActuatorSubsystem.ActuatorSubsystem;

public class PushActuator extends Command {
    ActuatorSubsystem actuatorSubsystem;
    double speed;

    public PushActuator(ActuatorSubsystem actuatorSubsystem, double speed){
        this.actuatorSubsystem = actuatorSubsystem;
        this.speed = speed;
        addRequirements(actuatorSubsystem);
    }

    @Override
    public void execute(){
        actuatorSubsystem.push(speed);
    }

    @Override
    public void end(boolean interrupted){
        actuatorSubsystem.stop();
    }
}

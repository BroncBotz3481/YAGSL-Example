package frc.robot.commands.swervedrive.claw;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;

public class MoveKicker extends Command {
    ClawSubsystem clawSubsystem;
    Double velocity;

    public MoveKicker(ClawSubsystem clawSubsystem, Double velocity){
        this.clawSubsystem = clawSubsystem;
        this.velocity = velocity;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        clawSubsystem.kickerMove(velocity);
    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stopKicker();
    }
}

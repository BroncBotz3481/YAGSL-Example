package frc.robot.commands.swervedrive.claw;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;

public class MoveKicker extends Command {
    ClawSubsystem kickerSubsystem;
    Double velocity;

    public MoveKicker(ClawSubsystem kickerSubsystem, Double velocity){
        this.kickerSubsystem = kickerSubsystem;
        this.velocity = velocity;
        addRequirements(kickerSubsystem);
    }

    @Override
    public void execute(){
        kickerSubsystem.kickerMove(velocity);
        System.out.println("moving kicker");
    }

    @Override
    public void end(boolean interrupted){
        kickerSubsystem.stopKicker();
        System.out.println("stoped moving kicker");
    }
}

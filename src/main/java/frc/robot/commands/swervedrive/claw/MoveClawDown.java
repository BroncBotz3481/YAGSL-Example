package frc.robot.commands.swervedrive.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;

public class MoveClawDown extends Command {
    ClawSubsystem clawSubsystem;
    double speed;

    public MoveClawDown(ClawSubsystem clawSubsystem, double speed){
        this.clawSubsystem = clawSubsystem;
        this.speed = speed;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        clawSubsystem.moveDown(speed);
    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stop();
        System.out.println("stoped moving claw down");
    }
}

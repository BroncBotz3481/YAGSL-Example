package frc.robot.commands.swervedrive.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;

public class MoveClawUp extends Command {
    ClawSubsystem clawSubsystem;
    double speed;

    public MoveClawUp(ClawSubsystem clawSubsystem, double speed){
        this.clawSubsystem = clawSubsystem;
        this.speed = speed;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        clawSubsystem.moveUp(speed);
    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stop();
        System.out.println("stoped moving claw up");
    }
}

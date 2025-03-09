package frc.robot.commands.swervedrive.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.FunnelSubsystem.FunnelSubsystem;

public class MoveFunnelUp extends Command {
    FunnelSubsystem funnelSubsystem;
    double speed;

    public MoveFunnelUp(FunnelSubsystem funnelSubsystem, double speed){
        this.funnelSubsystem = funnelSubsystem;
        this.speed = speed;
        addRequirements(funnelSubsystem);
    }

    @Override
    public void execute(){
        //if(funnelSubsystem.getEncoderValue() < 0.1){
            funnelSubsystem.moveUp(speed);
            //System.out.println(funnelSubsystem.getEncoderValue());
        //}
        //else{
            //funnelSubsystem.stop();
        //}

    }

    @Override
    public void end(boolean interrupted){
        funnelSubsystem.stop();
        System.out.println("stoped moving funnel up");
    }
}

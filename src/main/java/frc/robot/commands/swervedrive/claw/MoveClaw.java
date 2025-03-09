package frc.robot.commands.swervedrive.claw;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;

public class MoveClaw extends Command {
    ClawSubsystem clawSubsystem;
    DoubleSupplier velocity;

    public MoveClaw(ClawSubsystem clawSubsystem, DoubleSupplier velocity){
        this.clawSubsystem = clawSubsystem;
        this.velocity = velocity;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        clawSubsystem.move(velocity);
    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stop();
        System.out.println("stoped moving");
    }
}

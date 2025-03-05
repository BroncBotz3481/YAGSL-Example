package frc.robot.commands.swervedrive.Led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.LedSubsystem.LedSubsystem;

public class TurnRed extends Command {
    LedSubsystem ledSubsystem;

 public TurnRed(LedSubsystem ledSubsystem){
    this.ledSubsystem = ledSubsystem;

 }
 @Override
 public void execute(){
 
    ledSubsystem.turnRed();
    }
}//later note: there is no end so that might be the problem.


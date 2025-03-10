package frc.robot.commands.swervedrive.claw;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;
import frc.robot.utilities.Constants.AutoConstants;
import frc.robot.utilities.Constants.LimitConstants;

public class MoveClawHighCoral extends Command {
    ClawSubsystem clawSubsystem;

    public MoveClawHighCoral(ClawSubsystem clawSubsystem){
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute(){
        clawSubsystem.moveTo(AutoConstants.HIGH_CORAL_CLAW_POSITION + LimitConstants.CLAW_LOWER_LIMIT);
    }

    @Override
    public void end(boolean interrupted){
        clawSubsystem.stopClaw();
    }
}

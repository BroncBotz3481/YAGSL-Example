package frc.robot.commands.swervedrive.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import limelight.Limelight;
import limelight.networktables.LimelightSettings;
import limelight.networktables.LimelightSettings.LEDMode;

public class LimelightBlink extends Command {
    @Override
    public void initialize() {
        Limelight limelight = new Limelight("limelight-butler");
        LimelightSettings limelightSettings = new LimelightSettings(limelight);

        
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

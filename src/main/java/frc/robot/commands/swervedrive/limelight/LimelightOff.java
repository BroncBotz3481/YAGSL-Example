package frc.robot.commands.swervedrive.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import limelight.Limelight;
import limelight.networktables.LimelightSettings;
import limelight.networktables.LimelightSettings.LEDMode;

public class LimelightOff extends Command {
    @Override
    public void initialize() {
        Limelight limelight = new Limelight("limelight-butler");
        LimelightSettings limelightSettings = new LimelightSettings(limelight);

        limelightSettings.withLimelightLEDMode(LEDMode.ForceOff);
    }
}

package frc.robot.commands.swervedrive.Led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.LedSubsystem.LedSubsystem;


public class LedControll extends Command {
  private final LedSubsystem ledSubsystem;
  private final int r, g, b;

  public LedControll(LedSubsystem ledSubsystem, int r, int g, int b) {
      this.ledSubsystem = ledSubsystem;
      this.r = r;
      this.g = g;
      this.b = b;
      addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
      ledSubsystem.setColor(r, g, b);
  }

  @Override
  public boolean isFinished() {
      return false; // run continuously
  }
}
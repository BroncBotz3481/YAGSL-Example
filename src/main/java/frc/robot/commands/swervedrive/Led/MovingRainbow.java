package frc.robot.commands.swervedrive.Led;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.LedSubsystem.LedSubsystem;


public class MovingRainbow extends Command {
    private final LedSubsystem ledSubsystem;

    public MovingRainbow(LedSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        ledSubsystem.setMovingRainbow(); // Continuously update LEDs with shifting colors
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously as a default command
    }
}
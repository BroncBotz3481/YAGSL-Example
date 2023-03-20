package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import java.util.HashMap;

public class AutoMap {
  public final HashMap<String, Command> eventMap = new HashMap<>();

  public AutoMap() {

    eventMap.put("ScoreLow", new PrintCommand("Passed Event 1!"));
  }

  public HashMap<String, Command> getEventMap() {
    return eventMap;
  }
}
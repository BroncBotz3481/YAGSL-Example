package frc.robot.subsystems.swervedrive2.swervelib.parser;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.subsystems.swervedrive2.swervelib.SwerveDrive;
import java.io.File;
import java.io.IOException;

public class SwerveParser
{

  /**
   * Open JSON file.
   *
   * @param file JSON File to open.
   * @return JsonNode of file.
   */
  private static JsonNode openJson(File file)
  {
    try
    {
      return new ObjectMapper().readTree(file);
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * Create {@link SwerveDrive} from JSON configuration directory.
   *
   * @param directory JSON Configuration directory.
   * @return {@link SwerveDrive} instance.
   */
  public static SwerveDrive fromJSONDirectory(File directory)
  {
    return null;
  }
}

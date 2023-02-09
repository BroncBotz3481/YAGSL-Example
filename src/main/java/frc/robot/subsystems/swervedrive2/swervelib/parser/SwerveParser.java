package frc.robot.subsystems.swervedrive2.swervelib.parser;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swervedrive2.swervelib.SwerveDrive;
import frc.robot.subsystems.swervedrive2.swervelib.parser.json.ControllerPropertiesJson;
import frc.robot.subsystems.swervedrive2.swervelib.parser.json.ModuleJson;
import frc.robot.subsystems.swervedrive2.swervelib.parser.json.PIDFPropertiesJson;
import frc.robot.subsystems.swervedrive2.swervelib.parser.json.PhysicalPropertiesJson;
import frc.robot.subsystems.swervedrive2.swervelib.parser.json.SwerveDriveJson;
import java.io.File;
import java.io.IOException;

/**
 * Helper class used to parse the JSON directory with specified configuration options.
 */
public class SwerveParser
{

  /**
   * Parsed swervedrive.json
   */
  private final SwerveDriveJson          swerveDriveJson;
  /**
   * Parsed controllerproperties.json
   */
  private final ControllerPropertiesJson controllerPropertiesJson;
  /**
   * Parsed modules/pidfproperties.json
   */
  private final PIDFPropertiesJson       pidfPropertiesJson;
  /**
   * Parsed modules/physicalproperties.json
   */
  private final PhysicalPropertiesJson   physicalPropertiesJson;
  /**
   * Array holding the module jsons given in {@link SwerveDriveJson}.
   */
  private final ModuleJson[]             moduleJsons;

  /**
   * Construct a swerve parser. Will throw an error if there is a missing file.
   *
   * @param directory Directory with swerve configurations.
   */
  public SwerveParser(File directory) throws IOException
  {
    checkDirectory(directory);
    swerveDriveJson = new ObjectMapper().readValue(new File(directory, "swervedrive.json"), SwerveDriveJson.class);
    controllerPropertiesJson = new ObjectMapper().readValue(new File(directory, "controllerproperties.json"),
                                                            ControllerPropertiesJson.class);
    pidfPropertiesJson = new ObjectMapper().readValue(new File(directory, "modules/pidfproperties.json"),
                                                      PIDFPropertiesJson.class);
    physicalPropertiesJson = new ObjectMapper().readValue(new File(directory, "modules/physicalproperties.json"),
                                                          PhysicalPropertiesJson.class);
    moduleJsons = new ModuleJson[swerveDriveJson.modules.length];
    for (int i = 0; i < moduleJsons.length; i++)
    {
      File moduleFile = new File(directory, "modules/" + swerveDriveJson.modules[i]);
      assert moduleFile.exists();
      moduleJsons[i] = new ObjectMapper().readValue(moduleFile, ModuleJson.class);
    }
  }

  /**
   * Open JSON file.
   *
   * @param file JSON File to open.
   * @return JsonNode of file.
   */
  private JsonNode openJson(File file)
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
   * Check directory structure.
   *
   * @param directory JSON Configuration Directory
   */
  private void checkDirectory(File directory)
  {
    assert new File(directory, "swervedrive.json").exists();
    assert new File(directory, "controllerproperties.json").exists();
    assert new File(directory, "modules").exists() && new File(directory, "modules").isDirectory();
    assert new File(directory, "modules/pidfproperties.json").exists();
    assert new File(directory, "modules/physicalproperties.json").exists();
  }

  /**
   * Create {@link SwerveDrive} from JSON configuration directory.
   *
   * @return {@link SwerveDrive} instance.
   */
  public SwerveDrive createSwerveDrive()
  {
    double                      maxSpeedMPS          = Units.feetToMeters(swerveDriveJson.maxSpeed);
    SwerveModuleConfiguration[] moduleConfigurations = new SwerveModuleConfiguration[moduleJsons.length];
    for (int i = 0; i < moduleConfigurations.length; i++)
    {
      ModuleJson module = moduleJsons[i];
      moduleConfigurations[i] = module.createModuleConfiguration(pidfPropertiesJson.angle, pidfPropertiesJson.drive,
                                                                 maxSpeedMPS,
                                                                 physicalPropertiesJson.createPhysicalProperties(
                                                                     swerveDriveJson.optimalVoltage));
    }
    SwerveDriveConfiguration swerveDriveConfiguration = new SwerveDriveConfiguration(moduleConfigurations,
                                                                                     swerveDriveJson.imu.createIMU(),
                                                                                     maxSpeedMPS);

    return new SwerveDrive(swerveDriveConfiguration,
                           controllerPropertiesJson.createControllerConfiguration(swerveDriveConfiguration));
  }
}

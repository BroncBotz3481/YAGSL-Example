package swervelib.parser.deserializer;

/**
 * Create classes only if the vendor dep exists.
 */
public class ReflectionsManager
{

  /**
   * Vendors that supply their own vendordep to communicate with their products.
   */
  public enum VENDOR
  {
    /**
     * REVLib
     */
    REV,
    /**
     * CTRE Phoenix 5 and 6
     */
    PHOENIX5,
    PHOENIX6,
    /**
     * ThriftyLib
     */
    THRIFTYBOT,
    /**
     * StudicaLib
     */
    STUDICA
  }

  /**
   * Check if the vendordep exists.
   *
   * @param vendor Vendor to check for their library.
   * @return Boolean on existence of their library.
   */
  public static boolean checkIfVendorLibExists(VENDOR vendor)
  {
    try
    {
      Class<?> library;
      switch (vendor)
      {
        case REV ->
        {
          library = Class.forName("com.revrobotics.spark.SparkBase");
          break;
        }
        case PHOENIX6 ->
        {
          library = Class.forName("com.ctre.phoenix6.hardware.TalonFXS");
          break;
        }
        case PHOENIX5 ->
        {
          library = Class.forName("com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX");
          break;
        }
        case THRIFTYBOT ->
        {
          library = Class.forName("com.thethriftybot.ThriftyNova");
          break;
        }
        case STUDICA ->
        {
          library = Class.forName("com.studica.frc.AHRS");
          break;
        }
        default ->
        {
          return false;
        }
      }
    } catch (Exception e)
    {
      return false;
    }
    return true;
  }

  /**
   * Create objects if the vendordep exists. Throw an exception when they dont.
   *
   * @param v              Vendor to check if the vendordep exists.
   * @param className      Wrapper classname to create.
   * @param parameterTypes Parameter types for the wrappers constructor.
   * @param parameters     Parameters for the wrappers constructor
   * @param <T>            Wrapper type.
   * @return Wrapper object.
   */
  public static <T> T create(VENDOR v, String className, Class<?>[] parameterTypes, Object[] parameters)
  {
    if (!checkIfVendorLibExists(v))
    {
      throw new RuntimeException("Vendor " + v + " library not found! Please install it!");
    }
    try
    {
      Class<?> wrapper   = Class.forName(className);
      Object   vendorObj = wrapper.getDeclaredConstructor(parameterTypes).newInstance(parameters);
      return (T) vendorObj;
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

}

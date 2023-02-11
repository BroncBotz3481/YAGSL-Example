package frc.robot.subsystems.swervedrive2.swervelib.imu;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * Communicates with the NavX as the IMU.
 */
public class NavXSwerve extends SwerveIMU
{

  /**
   * NavX IMU.
   */
  private AHRS   gyro;
  /**
   * Offset for the NavX yaw reading.
   */
  private double yawOffset = 0;

  /**
   * Constructor for the NavX swerve.
   */
  public NavXSwerve()
  {
    try
    {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      gyro = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException ex)
    {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    gyro.reset();
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
  }

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Yaw angle in degrees.
   */
  @Override
  public void setYaw(double yaw)
  {
    gyro.reset();
    if (yaw != 0)
    {
      yawOffset = yaw;
    }
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray)
  {
    yprArray[0] = gyro.getYaw();
    yprArray[1] = gyro.getPitch();
    yprArray[2] = gyro.getRoll();
  }
}

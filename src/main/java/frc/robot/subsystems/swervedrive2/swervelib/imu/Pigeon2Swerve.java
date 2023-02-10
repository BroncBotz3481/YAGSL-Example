package frc.robot.subsystems.swervedrive2.swervelib.imu;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class Pigeon2Swerve extends SwerveIMU
{
  /**
   * Pigeon2 IMU device.
   */
  WPI_Pigeon2 imu;


  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid  CAN ID for the pigeon
   * @param canbus CAN Bus name the pigeon resides on.
   */
  public Pigeon2Swerve(int canid, String canbus)
  {
    imu = new WPI_Pigeon2(canid, canbus);
    Pigeon2Configuration config = new Pigeon2Configuration();
    imu.configAllSettings(config);
  }

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon
   */
  public Pigeon2Swerve(int canid)
  {
    this(canid, "");
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    imu.configFactoryDefault();
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
    imu.clearStickyFaults();
  }

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Angle in degrees.
   */
  @Override
  public void setYaw(double yaw)
  {
    imu.setYaw(yaw);
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU, inverts them all if SwerveIMU is inverted.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray)
  {
    imu.getYawPitchRoll(yprArray);
  }
}

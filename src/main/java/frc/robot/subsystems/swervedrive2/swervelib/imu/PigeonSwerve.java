package frc.robot.subsystems.swervedrive2.swervelib.imu;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class PigeonSwerve extends SwerveIMU
{

  WPI_PigeonIMU imu;

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon, does not support CANBus.
   */
  public PigeonSwerve(int canid)
  {
    imu = new WPI_PigeonIMU(canid);
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

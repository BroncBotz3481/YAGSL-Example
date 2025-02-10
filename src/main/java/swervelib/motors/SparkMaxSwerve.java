package swervelib.motors;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;

/** An implementation of {@link com.revrobotics.spark.SparkMax} as a {@link SwerveMotor}. */
public class SparkMaxSwerve extends SparkSwerve {

  /**
   * Initialize the swerve motor.
   *
   * @param motor The SwerveMotor as a SparkMax object.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   * @param motorType Motor type controlled by the {@link SparkMax} motor controller.
   */
  public SparkMaxSwerve(SparkMax motor, boolean isDriveMotor, DCMotor motorType) {
    super(motor, new SparkMaxConfig(), isDriveMotor, motorType);
  }

  /**
   * Initialize the {@link SwerveMotor} as a {@link SparkMax} connected to a Brushless Motor.
   *
   * @param id CAN ID of the SparkMax.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   * @param motorType Motor type controlled by the {@link SparkMax} motor controller.
   */
  public SparkMaxSwerve(int id, boolean isDriveMotor, DCMotor motorType) {
    this(new SparkMax(id, MotorType.kBrushless), isDriveMotor, motorType);
  }
}

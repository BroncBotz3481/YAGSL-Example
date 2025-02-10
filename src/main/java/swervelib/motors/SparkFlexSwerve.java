package swervelib.motors;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;

/** An implementation of {@link SparkFlex} as a {@link SwerveMotor}. */
public class SparkFlexSwerve extends SparkSwerve {

  /**
   * Initialize the swerve motor.
   *
   * @param motor The SwerveMotor as a SparkFlex object.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   * @param motorType {@link DCMotor} which the {@link SparkFlex} is attached to.
   */
  public SparkFlexSwerve(SparkFlex motor, boolean isDriveMotor, DCMotor motorType) {
    super(motor, new SparkFlexConfig(), isDriveMotor, motorType);
  }

  /**
   * Initialize the {@link SwerveMotor} as a {@link SparkFlex} connected to a Brushless Motor.
   *
   * @param id CAN ID of the SparkFlex.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   * @param motorType {@link DCMotor} which the {@link SparkFlex} is attached to.
   */
  public SparkFlexSwerve(int id, boolean isDriveMotor, DCMotor motorType) {
    this(new SparkFlex(id, MotorType.kBrushless), isDriveMotor, motorType);
  }
}

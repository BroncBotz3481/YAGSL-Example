package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.CLIMB_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_SPEED;
import static frc.robot.Constants.Tolerances.ELEVATOR_TOLERANCE;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax lClimb = new SparkMax(61, MotorType.kBrushless);
    private SparkMax rClimb = new SparkMax(62, MotorType.kBrushless);

    private SparkMaxConfig config = new SparkMaxConfig();

    public ClimbSubsystem() {
        config.idleMode(IdleMode.kBrake);
        lClimb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        rClimb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {        
        SmartDashboard.putNumber("Climb/lHeight", lClimb.getEncoder().getPosition());
        SmartDashboard.putNumber("Climb/rHeight", rClimb.getEncoder().getPosition());

        if (Math.abs(lClimb.getEncoder().getPosition() - rClimb.getEncoder().getPosition()) <= ELEVATOR_TOLERANCE) {
            lClimb.stopMotor();
            rClimb.stopMotor();
        }
        if (lClimb.getEncoder().getPosition() > CLIMB_LIMIT || rClimb.getEncoder().getPosition() > CLIMB_LIMIT) {
            lClimb.stopMotor();
            rClimb.stopMotor();
        }
    }

    public Command climb() {
        return Commands.run(() -> {
            lClimb.set(CLIMB_SPEED);
            rClimb.set(CLIMB_SPEED);
        }).handleInterrupt(() -> {
            lClimb.stopMotor();
            rClimb.stopMotor();
        });
    }
}

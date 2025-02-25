package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.HomeConstants.ALGAE_HOME_POSITION;
import static frc.robot.Constants.HomeConstants.CORAL_HOME_POSITION;
import static frc.robot.Constants.HomeConstants.ELEVATOR_HOME_POSITION;
import static frc.robot.Constants.IOSpeeds.ALGAE_INTAKE_SPEED;
import static frc.robot.Constants.IOSpeeds.ALGAE_SHOOT_SPEED;
import static frc.robot.Constants.IOSpeeds.CORAL_INTAKE_SPEED;
import static frc.robot.Constants.IOSpeeds.CORAL_SHOOT_SPEED;
import static frc.robot.Constants.PIDConstants.A_WRIST_D;
import static frc.robot.Constants.PIDConstants.A_WRIST_I;
import static frc.robot.Constants.PIDConstants.A_WRIST_P;
import static frc.robot.Constants.PIDConstants.C_WRIST_D;
import static frc.robot.Constants.PIDConstants.C_WRIST_I;
import static frc.robot.Constants.PIDConstants.C_WRIST_P;
import static frc.robot.Constants.PIDConstants.ELEVATOR_D;
import static frc.robot.Constants.PIDConstants.ELEVATOR_I;
import static frc.robot.Constants.PIDConstants.ELEVATOR_P;
import static frc.robot.Constants.ReefLevels.C_L1_POSITION;
import static frc.robot.Constants.ReefLevels.C_L2_POSITION;
import static frc.robot.Constants.ReefLevels.C_L3_POSITION;
import static frc.robot.Constants.ReefLevels.E_L1_POSITION;
import static frc.robot.Constants.ReefLevels.E_L2_POSITION;
import static frc.robot.Constants.ReefLevels.E_L3_POSITION;
import static frc.robot.Constants.Tolerances.ELEVATOR_TOLERANCE;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ArmSubsystem extends SubsystemBase {
    private SparkMax lElevator = new SparkMax(51, MotorType.kBrushless);
    private SparkMax rElevator = new SparkMax(52, MotorType.kBrushless);

    private SparkMax coralWrist = new SparkMax(31, MotorType.kBrushless);
    private SparkMax coralShooter = new SparkMax(32, MotorType.kBrushless);

    // Algae
    private SparkMax algaeWrist = new SparkMax(41, MotorType.kBrushless);
    private SparkMax lAlgaeIntake = new SparkMax(42, MotorType.kBrushless);
    private SparkMax rAlgaeIntake = new SparkMax(43, MotorType.kBrushless);

    private SparkMaxConfig config = new SparkMaxConfig();

    private double algaeWristPosition = 0;
    private double coralWristPosition = 0;
    private double elevatorPosition = 0;

    private Trigger badElevTrigger = new Trigger(() -> Math
            .abs(lElevator.getEncoder().getPosition() - rElevator.getEncoder().getPosition()) <= ELEVATOR_TOLERANCE);

    public ArmSubsystem() {
        badElevTrigger.onTrue(Commands.runOnce(() -> {
            /* TODO elastic notifications */}));
        config.idleMode(IdleMode.kBrake);
        coralShooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lAlgaeIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rAlgaeIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.closedLoop.pid(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);
        lElevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        rElevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.inverted(false);
        config.closedLoop.pid(C_WRIST_P, C_WRIST_I, C_WRIST_D);
        coralWrist.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.closedLoop.pid(A_WRIST_P, A_WRIST_I, A_WRIST_D);
        algaeWrist.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Shooter Velocity", coralShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("lAlgae Intake Velocity", lAlgaeIntake.getEncoder().getVelocity());
        SmartDashboard.putNumber("rAlgae Intake Velocity", rAlgaeIntake.getEncoder().getVelocity());
        SmartDashboard.putNumber("Coral Wrist Angle", coralWrist.getEncoder().getPosition());
        SmartDashboard.putNumber("Algae Wrist Angle", algaeWrist.getEncoder().getPosition());
        SmartDashboard.putNumber("lElevator Height", lElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("rElevator Height", rElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Coral Wrist Setpoint", coralWristPosition);
        SmartDashboard.putNumber("Algae Wrist Setpoint", algaeWristPosition);
        SmartDashboard.putNumber("Elevator Setpoint", elevatorPosition);

        coralWrist.getClosedLoopController().setReference(coralWristPosition, ControlType.kPosition);
        algaeWrist.getClosedLoopController().setReference(algaeWristPosition, ControlType.kPosition);

        if (!badElevTrigger.getAsBoolean()) {
            lElevator.getClosedLoopController().setReference(elevatorPosition, ControlType.kPosition);
            rElevator.getClosedLoopController().setReference(elevatorPosition, ControlType.kPosition);
        } else {
            lElevator.stopMotor();
            rElevator.stopMotor();
        }
    }

    public boolean isCoralIntaked() {
        return coralShooter.get() > 0.0 && MathUtil.isNear(0, coralShooter.getEncoder().getVelocity(), 2000);
    }

    public boolean isAlgaeIntaked() {
        return lAlgaeIntake.get() > 0.0 && MathUtil.isNear(0, lAlgaeIntake.getEncoder().getVelocity(), 2000);
    }

    public boolean isElevatorHomed() {
        return MathUtil.isNear(ELEVATOR_HOME_POSITION,
                (lElevator.getEncoder().getPosition() + rElevator.getEncoder().getPosition()) / 2.0, 2);
    }

    public boolean isCoralHomed() {
        return MathUtil.isNear(CORAL_HOME_POSITION, coralWrist.getEncoder().getPosition(), 1);
    }

    public boolean isAlgaeHomed() {
        return MathUtil.isNear(ALGAE_HOME_POSITION, algaeWrist.getEncoder().getPosition(), 1);
    }

    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(elevatorPosition,
                (lElevator.getEncoder().getPosition() + rElevator.getEncoder().getPosition()) / 2.0, 2);
    }

    public boolean isCoralAtPosition() {
        return MathUtil.isNear(coralWristPosition, coralWrist.getEncoder().getPosition(), 1);
    }

    public boolean isAlgaeAtPosition() {
        return MathUtil.isNear(algaeWristPosition, algaeWrist.getEncoder().getPosition(), 1);
    }

    public Command homeElevator() {
        return Commands.runOnce(() -> {
            elevatorPosition = ELEVATOR_HOME_POSITION;
        }).andThen(Commands.waitUntil(this::isElevatorHomed));
    }

    public Command homeCoral() {
        return Commands.runOnce(() -> {
            coralWristPosition = CORAL_HOME_POSITION;
        }).andThen(Commands.waitUntil(this::isCoralHomed));
    }

    public Command homeAlgae() {
        return Commands.runOnce(() -> {
            algaeWristPosition = ALGAE_HOME_POSITION;
        }).andThen(Commands.waitUntil(this::isAlgaeHomed));
    }

    public Command homeEverything() {
        return Commands.runOnce(() -> {
            homeCoral();
            homeElevator();
            homeAlgae();
        });
    }

    public Command intakeCoral() {
        return Commands.run(() -> {
            coralShooter.set(CORAL_INTAKE_SPEED);
        }).until(this::isCoralIntaked);
    }

    public Command intakeAlgae() {
        return Commands.run(() -> {
            lAlgaeIntake.set(ALGAE_INTAKE_SPEED);
            rAlgaeIntake.set(ALGAE_INTAKE_SPEED);
        }).until(this::isAlgaeIntaked);
    }

    public Command shootCoral() {
        return Commands.run(() -> {
            coralShooter.set(CORAL_SHOOT_SPEED);
        }).handleInterrupt(() -> {
            coralShooter.stopMotor();
        });
    }

    public Command shootAlgae() {
        return Commands.run(() -> {
            lAlgaeIntake.set(ALGAE_SHOOT_SPEED);
            rAlgaeIntake.set(ALGAE_SHOOT_SPEED);
        }).handleInterrupt(() -> {
            lAlgaeIntake.stopMotor();
            rAlgaeIntake.stopMotor();
        });
    }

    public Command extendTo(double pos) {
        return Commands.runOnce(() -> {
            elevatorPosition = pos;
        }).andThen(Commands.waitUntil(this::isElevatorAtPosition));
    }

    public Command coralTo(double pos) {
        return Commands.runOnce(() -> {
            coralWristPosition = pos;
        }).andThen(Commands.waitUntil(this::isCoralAtPosition));
    }

    public Command algaeTo(double pos) {
        return Commands.runOnce(() -> {
            algaeWristPosition = pos;
        }).andThen(Commands.waitUntil(this::isAlgaeAtPosition));
    }

    public Command extendL1() {
        return Commands.runOnce(() -> {
            elevatorPosition = E_L1_POSITION;
            coralWristPosition = C_L1_POSITION;
        }).andThen(Commands.waitUntil(this::isElevatorAtPosition));
    }

    public Command scoreL1() {
        return extendL1().andThen(shootCoral()).withTimeout(1).andThen(homeElevator()).alongWith(homeCoral());
    }

    public Command extendL2() {
        return Commands.runOnce(() -> {
            elevatorPosition = E_L2_POSITION;
            coralWristPosition = C_L2_POSITION;
        });
    }

    public Command scoreL2() {
        return extendL2().andThen(shootCoral()).withTimeout(1).andThen(homeElevator()).alongWith(homeCoral());
    }

    public Command extendL3() {
        return Commands.runOnce(() -> {
            elevatorPosition = E_L3_POSITION;
            coralWristPosition = C_L3_POSITION;
        });
    }

    public Command scoreL3() {
        return extendL3().andThen(shootCoral()).withTimeout(1).andThen(homeElevator()).alongWith(homeCoral());
    }

    public Command scoreCoral(int level) {
        switch (level) {
            case 1:
                return scoreL1();
            case 2:
                return scoreL2();
            case 3:
                return scoreL3();
            default:
                return Commands.none();
        }
    }
}

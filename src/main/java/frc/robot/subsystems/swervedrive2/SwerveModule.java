package frc.robot.subsystems.swervedrive2;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.Drivebase.DriveFeedforwardGains;
import frc.robot.Constants.Drivebase.DrivetrainLimitations;
import frc.robot.Constants.Drivebase.EncoderConversions;
import frc.robot.Constants.Drivebase.ModulePIDFGains;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive2.math.BetterSwerveModuleState;

public class SwerveModule
{

    public        int                   moduleNumber;
    private final double                angleOffset;
    private final CANSparkMax           angleMotor;
    private final CANSparkMax           driveMotor;
    private final CANCoder              absoluteEncoder;
    private final RelativeEncoder       angleEncoder;
    private final RelativeEncoder       driveEncoder;
    private final SparkMaxPIDController angleController;
    private final SparkMaxPIDController driveController;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveFeedforwardGains.KS, DriveFeedforwardGains.KV,
                                                                    DriveFeedforwardGains.KA);
    private       double                lastAngle;
    private       double                angle, omega, speed, fakePos, lastTime, dt;
    private Timer time;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants)
    {
        angle = 0;
        speed = 0;
        omega = 0;
        fakePos = 0;
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        driveMotor.restoreFactoryDefaults();

        // Config angle encoders
        absoluteEncoder = new CANCoder(moduleConstants.cancoderID);
        absoluteEncoder.configFactoryDefault();
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.sensorDirection = Drivebase.CANCODER_INVERT;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
        absoluteEncoder.configAllSettings(canCoderConfiguration);

        angleEncoder = angleMotor.getEncoder();
        angleEncoder.setPositionConversionFactor(EncoderConversions.DEGREES_PER_STEERING_ROTATION);
        angleEncoder.setVelocityConversionFactor(EncoderConversions.DEGREES_PER_STEERING_ROTATION / 60);
        angleEncoder.setPosition(absoluteEncoder.getAbsolutePosition() - angleOffset);

        // Config angle motor/controller
        angleController = angleMotor.getPIDController();
        angleController.setP(ModulePIDFGains.MODULE_KP);
        angleController.setI(ModulePIDFGains.MODULE_KI);
        angleController.setD(ModulePIDFGains.MODULE_KD);
        angleController.setFF(ModulePIDFGains.MODULE_KF);
        angleController.setIZone(ModulePIDFGains.MODULE_IZ);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMaxInput(180);
        angleController.setPositionPIDWrappingMinInput(-180);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        // Config drive motor/controller
        driveController = driveMotor.getPIDController();
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(EncoderConversions.METERS_PER_MOTOR_ROTATION);
        driveEncoder.setVelocityConversionFactor(EncoderConversions.METERS_PER_MOTOR_ROTATION / 60);
        driveController.setP(ModulePIDFGains.VELOCITY_KP);
        driveController.setI(ModulePIDFGains.VELOCITY_KI);
        driveController.setD(ModulePIDFGains.VELOCITY_KD);
        driveController.setFF(ModulePIDFGains.VELOCITY_KF);
        driveController.setIZone(ModulePIDFGains.VELOCITY_IZ);
        driveMotor.setInverted(Drivebase.DRIVE_MOTOR_INVERT);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveMotor.burnFlash();
        angleMotor.burnFlash();

        lastAngle = getState().angle.getDegrees();

        if (!Robot.isReal())
        {
            time = new Timer();
            time.start();
            lastTime = time.get();
        }
    }

    public void setDesiredState(BetterSwerveModuleState desiredState, boolean isOpenLoop)
    {
        SwerveModuleState simpleState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        simpleState = SwerveModuleState.optimize(simpleState, getState().angle);
        desiredState = new BetterSwerveModuleState(simpleState.speedMetersPerSecond, simpleState.angle,
                                                   desiredState.omegaRadPerSecond);

        SmartDashboard.putNumber("Optimized " + moduleNumber + " Speed Setpoint: ", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Optimized " + moduleNumber + " Angle Setpoint: ", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Module " + moduleNumber + " Omega: ", Math.toDegrees(desiredState.omegaRadPerSecond));

        if (isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / DrivetrainLimitations.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else
        {
            double velocity = desiredState.speedMetersPerSecond;
            driveController.setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(velocity));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DrivetrainLimitations.MAX_SPEED * 0.01) ?
                        lastAngle :
                        desiredState.angle.getDegrees()); // Prevents module rotation if speed is less than 1%
        angleController.setReference(angle, ControlType.kPosition, 0,
                                     Math.toDegrees(desiredState.omegaRadPerSecond) * ModulePIDFGains.MODULE_KV);
        lastAngle = angle;

        this.angle = desiredState.angle.getDegrees();
        omega = desiredState.omegaRadPerSecond;
        speed = desiredState.speedMetersPerSecond;

        if (!Robot.isReal())
        {
            dt = time.get() - lastTime;
            fakePos += (speed * dt);
            lastTime = time.get();
        }
    }

    public BetterSwerveModuleState getState()
    {
        double     velocity;
        Rotation2d azimuth;
        double     omega;
        if (Robot.isReal())
        {
            velocity = driveEncoder.getVelocity();
            azimuth = Rotation2d.fromDegrees(angleEncoder.getPosition());
            omega = angleEncoder.getVelocity();
        } else
        {
            velocity = speed;
            azimuth = Rotation2d.fromDegrees(this.angle);
            omega = this.omega;
        }
        return new BetterSwerveModuleState(velocity, azimuth, omega);
    }

    public SwerveModulePosition getPosition()
    {
        double     position;
        Rotation2d azimuth;
        if (Robot.isReal())
        {
            position = driveEncoder.getPosition();
            azimuth = Rotation2d.fromDegrees(angleEncoder.getPosition());
        } else
        {
            position = fakePos;
            azimuth = Rotation2d.fromDegrees(angle + (Math.toDegrees(omega) * dt));
        }
        SmartDashboard.putNumber("Module " + moduleNumber + "Angle", azimuth.getDegrees());
        return new SwerveModulePosition(position, azimuth);
    }

    public double getCANCoder()
    {
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getRelativeEncoder()
    {
        return angleEncoder.getPosition();
    }

    public void setMotorBrake(boolean brake)
    {
        driveMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}

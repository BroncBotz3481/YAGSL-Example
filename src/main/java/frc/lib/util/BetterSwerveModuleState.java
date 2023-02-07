package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class BetterSwerveModuleState extends SwerveModuleState {
    public double speedMetersPerSecond;

    /** Rad per sec */
    public double omegaRadPerSecond = 0;

    public Rotation2d angle = Rotation2d.fromDegrees(0);

    /** Constructs a SwerveModuleState with zeros for speed and angle. */
    public BetterSwerveModuleState() {}

    /**
     * Constructs a SwerveModuleState.
     *
     * @param speedMetersPerSecond The speed of the wheel of the module.
     * @param angle The angle of the module.
     * @param omegaRadPerSecond The angular velocity of the module.
     */
    public BetterSwerveModuleState(double speedMetersPerSecond, Rotation2d angle, double omegaRadPerSecond) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angle;
        this.omegaRadPerSecond = omegaRadPerSecond;
    }
}

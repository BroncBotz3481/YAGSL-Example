package swervelib.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ironmaple.simulation.motorsims.ControlRequest;

/**
 * Class that wraps around {@link org.ironmaple.simulation.drivesims.SwerveModuleSimulation}
 */
public class SwerveModuleSimulation
{

  private org.ironmaple.simulation.drivesims.SwerveModuleSimulation mapleSimModule = null;

  /**
   * Configure the maple sim module
   *
   * @param mapleSimModule the {@link org.ironmaple.simulation.drivesims.SwerveModuleSimulation} object for simulation
   */
  public void configureSimModule(org.ironmaple.simulation.drivesims.SwerveModuleSimulation mapleSimModule)
  {
    this.mapleSimModule = mapleSimModule;
    mapleSimModule.getDriveMotorConfigs()
                  .withDefaultFeedForward(Volts.zero())
                  .withVelocityVoltageController(Volts.per(RPM).ofNative(7.0 / 3000.0), true);
  }

  /**
   * Update the position and state of the module. Called from {@link swervelib.SwerveModule#setDesiredState} function
   * when simulated.
   *
   * @param desiredState State the swerve module is set to.
   */
  public void updateStateAndPosition(SwerveModuleState desiredState)
  {
    mapleSimModule.requestSteerControl(new ControlRequest.PositionVoltage(desiredState.angle.getMeasure()));
    mapleSimModule.requestDriveControl(new ControlRequest.VelocityVoltage(
        RadiansPerSecond.of(desiredState.speedMetersPerSecond / mapleSimModule.WHEEL_RADIUS.in(Meters))
    ));
  }

  /**
   * Get the simulated swerve module position.
   *
   * @return {@link SwerveModulePosition} of the simulated module.
   */
  public SwerveModulePosition getPosition()
  {
    return new SwerveModulePosition(
        mapleSimModule.getDriveWheelFinalPosition().in(Radian) * mapleSimModule.WHEEL_RADIUS.in(Meters),
        mapleSimModule.getSteerAbsoluteFacing()
    );
  }

  /**
   * Get the {@link SwerveModuleState} of the simulated module.
   *
   * @return {@link SwerveModuleState} of the simulated module.
   */
  public SwerveModuleState getState()
  {
    if (mapleSimModule == null)
    {
      return new SwerveModuleState();
    }
    SwerveModuleState state = mapleSimModule.getCurrentState();
    state.angle = state.angle.minus(new Rotation2d());
    return state;
  }
}

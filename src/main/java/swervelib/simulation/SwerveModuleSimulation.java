package swervelib.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ironmaple.simulation.motorsims.ControlRequest;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

/**
 * Class that wraps around {@link org.ironmaple.simulation.drivesims.SwerveModuleSimulation}
 */
public class SwerveModuleSimulation
{
  private Optional<org.ironmaple.simulation.drivesims.SwerveModuleSimulation> mapleSimModule = Optional.empty();

  /**
   * Create simulation class
   */
  public SwerveModuleSimulation() {}

  /**
   * Configures the maple-sim SwerveModuleSimulation instance
   * */
  public void setMapleSimModule(org.ironmaple.simulation.drivesims.SwerveModuleSimulation mapleSimModule)
  {
    this.mapleSimModule = Optional.of(mapleSimModule);
    mapleSimModule
            .getSteerMotorConfigs()
            .withPositionVoltageController(
                    Volts.per(Degrees).ofNative(8.0/(180.0 * 12)), Volts.per(RotationsPerSecond).ofNative(0)
            );
  }

  /**
   * Update the position and state of the module. Called from {@link swervelib.SwerveModule#setDesiredState} function
   * when simulated.
   *
   * @param desiredState State the swerve module is set to.
   */
  public void updateStateAndPosition(SwerveModuleState desiredState)
  {
    if (mapleSimModule.isPresent())
    {
      mapleSimModule.get().requestSteerControl(new ControlRequest.PositionVoltage(desiredState.angle.getMeasure()));
    }
  }

  /**
   * Get the simulated swerve module position.
   *
   * @return {@link SwerveModulePosition} of the simulated module.
   */
  public SwerveModulePosition getPosition()
  {
    if (mapleSimModule.isPresent())
    {
      return new SwerveModulePosition(
              mapleSimModule.get().getDriveWheelFinalPositionRad() * mapleSimModule.get().WHEEL_RADIUS_METERS,
              mapleSimModule.get().getSteerAbsoluteFacing()
      );
    }
    return new SwerveModulePosition(0, new Rotation2d());
  }

  /**
   * Get the {@link SwerveModuleState} of the simulated module.
   *
   * @return {@link SwerveModuleState} of the simulated module.
   */
  public SwerveModuleState getState()
  {
    if (mapleSimModule.isPresent())
    {
      return mapleSimModule.get().getCurrentState();
    }

    return new SwerveModuleState();
  }
}

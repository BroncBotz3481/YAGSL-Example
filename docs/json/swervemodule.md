# Swerve Module Configuration (`module/x.json`)

The swerve module configuration configures unique properties of each swerve module. It maps 1:1
with [`ModuleJson.java`](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/parser/json/ModuleJson.java)
which is used to
create [`SwerveModuleConfiguration`](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/parser/SwerveModuleConfiguration.java).
This configuration file interacts directly with swerve kinematics.

# Fields

| Name                    | Units                       | Required | Description                                                               |
|-------------------------|-----------------------------|----------|---------------------------------------------------------------------------|
| drive                   | [Device](devicejson.md)     | Y        | Drive motor device configuration.                                         |
| angle                   | [Device](devicejson.md)     | Y        | Angle motor device configuration.                                         |
| encoder                 | [Device](devicejson.md)     | Y        | Absolute encoder device configuration.                                    |
| inverted                | [MotorConfig](#MotorConfig) | Y        | Inversion state of each motor as a boolean.                               |
| absoluteEncoderOffset   | Degrees                     | Y        | Absolute encoder offset from 0 in degrees.                                |
| absoluteEncoderInverted | Bool                        | N        | Inversion state of the Absolute Encoder.                                  |
| location                | [MotorConfig](#MotorConfig) | Y        | The location of the swerve module from the center of the robot in inches. |
| useCosineCompensator    | Bool                        | N        | Whether or not to modulate drive motors when pointed in wrong direction (defaults to True) |

### MotorConfig

| Name  | Units | Required | Description        |
|-------|-------|----------|--------------------|
| drive | Value | Y        | Drive motor value. |  
| angle | Value | Y        | Angle motor value. |  

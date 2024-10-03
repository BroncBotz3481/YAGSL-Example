# Swerve Module Physical Properties (`physicalproperties.json`)

This JSON configures the physical properties shared with all the Swerve Modules. It maps 1:1
with [`PhysicalPropertiesJson.java`](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/parser/json/PhysicalPropertiesJson.java)
which creates
[`SwerveModulePhysicalCharacteristics.java`](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/parser/SwerveModulePhysicalCharacteristics.java).

# Fields

| Name                           | Units                             | Required | Description                                                                                                         |
|--------------------------------|-----------------------------------|----------|---------------------------------------------------------------------------------------------------------------------|
| wheelDiameter                  | Inches                            | Y        | Module wheel diameters in inches.                                                                                   |
| gearRatio                      | [MotorConfig](#MotorConfig)       | Y        | Gear ratio for the motors, number of times the motor has to spin before the wheel rotates a single time.            |
| encoderPulsePerRotation        | [MotorConfig](#MotorConfig)       | N        | Encoder pulse per rotation for non-integrated encoders. 1 for integrated encoders.                                  |
| currentLimit                   | [MotorConfig](#MotorConfig)       | N        | The current limit in AMPs to apply to the motors.                                                                   |
| rampRate                       | [MotorConfig](#MotorConfig)       | N        | The minimum number of seconds to take for the motor to go from 0 to full throttle.                                  |
| wheelGripCoefficientOfFriction | Coefficient of Friction on Carpet | N        | The grip tape coefficient of friction on carpet. Used to calculate the practical maximum acceleration.              |
| moduleFeedForwardClosedLoop    | Feedforward between `[-1, 0]`     | N        | The feedforward scalar to apply for 2nd order kinematics. If robot arcs while translating and rotating negate this. |

### MotorConfig

| Name  | Units  | Required | Description        |
|-------|--------|----------|--------------------|
| drive | Number | Y        | Drive motor value. |
| angle | Number | Y        | Angle motor value. |

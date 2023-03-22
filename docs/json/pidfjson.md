# PIDF Configuration

The PIDF configurations map 1:1
with [`PIDFConfig.java`](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/parser/PIDFConfig.java)
which stores information regarding the PID or PIDF configurations for the robot, such as module
velocity & position, and robot heading. Not every parameter is used on every PIDF configuration. For
example `heading` only takes into account the PID.

# Fields

A `0` in any place basically disables that portion of the PIDF.

| Name    | Units           | Required | Description                                                                                                     |
|---------|-----------------|----------|-----------------------------------------------------------------------------------------------------------------|
| p       | kP Gain         | Y        | Proportional Gain for the PID.                                                                                  |
| i       | kI Gain         | Y        | Integral Gain for the PID.                                                                                      |
| d       | kD Gain         | Y        | Derivative Gain for the PID.                                                                                    |
| f       | Number          | N        | Feedforward for the PID.                                                                                        |
| iz      | Number          | N        | Integral zone for the integrator of the PID.                                                                    |
| output  | [Range](#Range) | N        | The output range for the PID.                                                                                   |

### Range

| Name | Units  | Required | Description                                           |
|------|--------|----------|-------------------------------------------------------|
| min  | Number | N        | The minimum value in the PID range. Defaults to `-1`. |
| max  | Number | N        | The maximum value in the PID range. Defaults to `1`.  |

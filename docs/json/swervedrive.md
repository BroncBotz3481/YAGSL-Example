# Swerve Drive (`swervedrive.json`)

The Swerve Drive JSON configuration file configures everything related to the overall Swerve Drive.
It maps 1:1
to [`SwerveDriveJson.java`](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/parser/json/SwerveDriveJson.java)
which creates
a [`SwerveDriveConfiguration`](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/parser/SwerveDriveConfiguration.java)
that is used to create
the [`SwerveDrive`](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/SwerveDrive.java)
object.

# JSON Fields

| Name           | Units                   | Required | Description                                                                |
|----------------|-------------------------|----------|----------------------------------------------------------------------------|
| maxSpeed       | Feet Per Second         | Y        | Maximum robot speed in feet per second.                                    |
| optimalVoltage | Voltage                 | Y        | Optimal voltage to compensate to and base feedforward calculations off of. |
| imu            | [Device](devicejson.md) | Y        | Robot IMU used to determine heading of the robot.                          |
| invertedIMU    | Boolean                 | Y        | Inversion state of the IMU.                                                |
| modules        | String array            | Y        | Module JSONs in order clockwise order starting from front left.            |

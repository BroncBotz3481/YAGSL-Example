# Device Configuration

All devices in a swerve drive come down to a basic set of fields. The device configuration is used
to store and create those devices during parsing with a 1:1 mapping
to [DeviceJson.java](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/parser/json/DeviceJson.java).

# Fields

| Name   | Units                                                                                                                                                                                                                                                                                    | Required | Description                                                                                         |
|--------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------|-----------------------------------------------------------------------------------------------------|
| type   | `integrated`, `attached`, `analog`, `thrifty`, `throughbore`, `analog`, `dutycycle`, `cancoder`, `none` for Encoders. <br/><br/>`navx`, `pigeon`, `pigeon2`, `analog`, `adxrs450`, `adis16470`, `adis16448` for IMU's. <br/><br/>`sparkmax`, `falcon`, `talonfx`, `talonsrx` for motors. | Y        | The device type which is used for creation of the Swerve type.                                      |
| id     | Integer                                                                                                                                                                                                                                                                                  | Y        | The ID of the device on the CANBus, or the pin ID on the roboRIO for certain devices.               |
| canbus | String                                                                                                                                                                                                                                                                                   | N        | The canbus to instantiate the device on. Only works on devices compatible with alternate CAN buses. |

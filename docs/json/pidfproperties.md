# Swerve Module PID Configuration (`pidfpropreties.json`)

This file configures the PIDF values with integral zone and maximum output of the drive and motor
modules for every swerve module. It maps 1:1
to [`PIDFPropertiesJson.java`](../../src/main/java/frc/robot/subsystems/swervedrive/swervelib/parser/json/PIDFPropertiesJson.java)
and is used while initializing the `SwerveDriveConfiguration` object.

# Fields

| Name  | Units                                    | Required | Description                                                           |
|-------|------------------------------------------|----------|-----------------------------------------------------------------------|
| drive | [PIDF](pidfjson.md) | Y        | The configuration which will be used for the PIDF on the drive motor. |
| angle | [PIDF](pidfjson.md) | Y        | The configuration which will be used for the PIDF on the angle motor. |

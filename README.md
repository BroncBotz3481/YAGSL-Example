# Yet Another Generic Swerve Library (YAGSL) Example project

YAGSL is intended to be an easy implementation of a generic swerve drive that should work for most
square swerve drives. The project is documented
on [here](https://github.com/BroncBotz3481/YAGSL/wiki). The JSON documentation can also be
found [here](docs/START.md)

This example is intended to be a starting place on how to use YAGSL. By no means is this intended to
be the base of your robot project. YAGSL provides an easy way to generate a SwerveDrive which can be
used in both TimedRobot and Command-Based Robot templates.

# Overview

### Installation

Vendor URL:

```
https://broncbotz3481.github.io/YAGSL-Lib/yagsl/yagsl.json
```

[Javadocs here](https://broncbotz3481.github.io/YAGSL/)  
[Library here](https://github.com/BroncBotz3481/YAGSL/)  
[Code here](https://github.com/BroncBotz3481/YAGSL/tree/main/swervelib)  
[WIKI](https://github.com/BroncBotz3481/YAGSL/wiki)  
[Config Generation](https://broncbotz3481.github.io/YAGSL-Example/)

# Create an issue if there is any errors you find!

We will be actively montoring this and fix any issues when we can!

## Development

* Development happens here on `YAGSL-Example`. `YAGSL` and `YAGSL-Lib` are updated on a nightly
  basis.

### TL;DR Generate and download your configuration [here](https://broncbotz3481.github.io/YAGSL-Example/) and unzip it so that it follows structure below:

```text
deploy
└── swerve
    ├── controllerproperties.json
    ├── modules
    │   ├── backleft.json
    │   ├── backright.json
    │   ├── frontleft.json
    │   ├── frontright.json
    │   ├── physicalproperties.json
    │   └── pidfproperties.json
    └── swervedrive.json
```

### Then create your SwerveDrive object like this.

```java
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

File swerveJsonDirectory=new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive swerveDrive=new SwerveParser(swerveJsonDirectory).createSwerveDrive();
```

### Falcon Support would not have been possible without support from Team 1466 Webb Robotics!

# Configuration Tips

### My Robot Spins around uncontrollably during autonomous or when attempting to set the heading!

* Invert the gyro scope.
* Invert the drive motors for every module. (If front and back become reversed when turning)

### Angle motors are erratic.

* Invert the angle motor.

### My robot is heavy.

* Implement momentum velocity limitations in SwerveMath.

### Ensure the IMU is centered on the robot
# Yet Another Generic Swerve Library (YAGSL) Example project

YAGSL is intended to be an easy implementation of a generic swerve drive that should work for most
square swerve drives. The project is documented
on [here](https://github.com/BroncBotz3481/YAGSL/wiki). The JSON documentation can also be
found [here](docs/README.md)

This example is intended to be a starting place on how to use YAGSL. By no means is this intended to
be the base of your robot project. YAGSL provides an easy way to generate a SwerveDrive which can be
used in both TimedRobot and Command-Based Robot templates.

# Create an issue if there is any errors you find!

We will be actively montoring this and fix any issues when we can!

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
# How to create a swerve drive?

YAGSL is unique in the fact that you can create a swerve drive based entirely off of JSON
configuration files. The JSON configuration files should be located in
the [`deploy`](../src/main/deploy) directory. You can also create the Configuration objects manually
and instantiate your Swerve Drive that way.

## How to create a SwerveDrive using JSON.

This example program creates
the [`SwerveDrive`](../src/main/java/frc/robot/subsystems/swervedrive/swervelib/SwerveDrive.java)
in the [`SwerveSubsystem`](../src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java),
as you should only interact with it in the SwerveSubsystem if you are using command based
programming.

```java
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

File swerveJsonDirectory=new File(Filesystem.getDeployDirectory(),"swerve");
        SwerveDrive swerveDrive=new SwerveParser(swerveJsonDirectory).createSwerveDrive();
```

This way is fast and easy, no more large unmaintainable and daunting constants file to worry about!
To create a JSON directory look at the [JSON documentation](JSON.md).

# Creating a swerve drive.

- [ ] Install NavX, CTRE and REV vendor dependencies.
- [ ] Clone YAGSL. `git clone https://github.com/BroncBotz3481/YAGSL`
- [ ] Move [swervelib](https://github.com/BroncBotz3481/YAGSL/tree/main/swervelib)
  into `src/main/frc/java/subsystems/swervedrive/swerve`
- [ ] Copy [example JSON directory](https://github.com/BroncBotz3481/YAGSL/tree/main/deploy) or
  create your own then move it into `src/main/frc/deploy/swerve`
- [ ] Create SwerveDrive from JSON directory
  via `SwerveDrive drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();`
- [ ] View the java docs in [docs/index.html](https://broncbotz3481.github.io/YAGSL/)
- [ ] Experiment!

# Contributions

This library is based off of
95's [SwervyBot code](https://github.com/first95/FRC2023/tree/main/SwervyBot) in addition to a wide
variety of other code bases.
A huge thank you to every team which has open sourced their swerve code!

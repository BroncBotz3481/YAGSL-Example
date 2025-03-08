package frc.robot.commands.swervedrive.drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utilities.NetworkTables;

public class LimelightAlign extends Command{
    
    private SwerveSubsystem driveBase;
    private double desiredAngle;

    public LimelightAlign(SwerveSubsystem swerve) {
        driveBase = swerve;
        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        double tx = NetworkTables.getTx();
        System.out.println(tx);
        desiredAngle = driveBase.getHeading().getDegrees() + tx;
        driveBase.drive(0, 0, Math.toRadians(desiredAngle));

        //System.out.println(tx);
    }

    // @Override 
    // public boolean isFinished(){

    //      if(NetworkTables.getTv() && Math.abs(NetworkTables.getTx()) < 1) {
    //         return true;
    //     }

    //     return false;

    // }

    

    
}

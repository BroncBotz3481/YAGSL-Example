package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utilities.NetworkTables;

public class LimelightDriveAlignCommand extends Command {
    private SwerveSubsystem driveBase;
    private double desiredAngle;
    private double offsetX;
    private double offsetZ;
    private static double desiredDistanceZ = -1;
    private static double desiredDistanceX = 0;
    private double actualDistanceZ;
    private double actualDistanceX;

    // Uses offsetX and offsetY to change the position of the alignment in robotcontainer and not have to create multiple commands for alignment
    public LimelightDriveAlignCommand(SwerveSubsystem swerve, double offsetX, double offsetZ) {
        driveBase = swerve;
        this.offsetX = offsetX;
        this.offsetZ = offsetZ;
        addRequirements(driveBase);
    }

    @Override
    public void initialize() {
        System.out.println("***ran auto amp command***");
    }

    @Override
    public void execute() {

        var botpose = NetworkTables.getBotPos();

        // Check connection to Network table
        if (botpose.length == 0) {
            System.out.println(botpose.length);
            //System.out.println("No bot Pose");
            driveBase.drive(0, 0, driveBase.getHeading().getRadians());
            return;
        }
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) != 1){
            System.out.println("no traget");
            driveBase.drive(0, 0, driveBase.getHeading().getRadians());
            return;
        }



        // Check if able to see April Tag and if close to target stop moving and if not
        // seek
        // if (botpose[0] == 0 && botpose[1] == 0 && botpose[2] == 0) {
        //     if (closeToTarget == true) {
        //         driveBase.drive(0, 0, driveBase.getHeading().getRadians());
        //     }
        //     // else{
        //     // driveBase.drive(0, 0, driveBase.getHeading().getRadians() +
        //     // Math.toRadians(25) * turnMagnitude);
        //     // }
        //     return;
        // }
        desiredDistanceX = offsetX;
        desiredDistanceZ = offsetZ;
        System.out.println("distance: " + botpose[2] + " | angle: " + NetworkTables.getTx());
        actualDistanceX = botpose[0];
        actualDistanceZ = botpose[2];
        double distanceToMoveZ = (actualDistanceZ - desiredDistanceZ);
        double distanceToMoveX = -1 * (actualDistanceX - desiredDistanceX);
        desiredAngle = driveBase.getHeading().getDegrees() + actualDistanceX; // !!!!!!!CHANGED FROM NetworkTables.getTx() !!!!!!!
        driveBase.drive(distanceToMoveZ * 0.6, distanceToMoveX * 0.6, Math.toRadians(desiredAngle));
        
    }

    @Override
    public boolean isFinished() {

        // return Math.abs( currentAngle) <= 0.1d;
        if (Math.abs(actualDistanceX) <= 3d && Math.abs(desiredDistanceZ) - 0.25 <= Math.abs(actualDistanceZ) && Math.abs(actualDistanceZ) <= Math.abs(desiredDistanceZ) + 0.5) {
            System.out.println("Finished alignment");
            return true;
        }
        return false;
    }

    // public double getTx() {

    //     return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    // }

    // private double getZpos() {
    //     var botpose = NetworkTables.getBotPos();
    //     if (botpose.length == 0) {        
    //         return desiredDistanceZ;
    //     }
    //     return NetworkTables.getBotPos()[2];
    // }       
    
}

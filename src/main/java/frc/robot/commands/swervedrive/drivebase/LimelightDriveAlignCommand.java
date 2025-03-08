package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utilities.NetworkTables;

public class LimelightDriveAlignCommand extends Command {
    private SwerveSubsystem driveBase;
    private double desiredAngle;
    private static final double desiredDistanceZ = -1;
    private double actualDistanceZ;
    private double actualDistanceX;

    public LimelightDriveAlignCommand(SwerveSubsystem swerve) {
        driveBase = swerve;
        addRequirements(driveBase);
    }

    @Override
    public void initialize() {
        System.out.println("***ran auto amp command***");
        // // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(botpose);
        // // desiredAngle = botpose[4];
        // // double tx = getTx();
        // // desiredAngle = driveBase.getHeading().getDegrees() - tx;
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

        System.out.println("distance: " + botpose[2] + " | angle: " + NetworkTables.getTx());
        actualDistanceX = botpose[0];
        actualDistanceZ = botpose[2];
        double distanceToMoveZ = (actualDistanceZ - desiredDistanceZ);
        double distanceToMoveX = -1 * (actualDistanceX);
        desiredAngle = driveBase.getHeading().getDegrees() + actualDistanceX; // !!!!!!!CHANGED FROM NetworkTables.getTx() !!!!!!!
        //driveBase.drive(-Math.max(distanceToMoveZ * 0.6, 0.2), Math.max(distanceToMoveX * 0.6, 0.2), Math.toRadians(desiredAngle));
        driveBase.drive(distanceToMoveZ * 0.6, distanceToMoveX * 0.6, Math.toRadians(desiredAngle));
        // currentAngle = botpose[4];
        //double driveZ = Math.abs(distanceToMoveZ * 0.6) > 0.2 ? distanceToMoveZ * 0.6 : 0.2 * Math.signum(distanceToMoveZ);
        //double driveX = Math.abs(distanceToMoveX * 0.6) > 0.2 ? distanceToMoveX * 0.6 : 0.2 * Math.signum(distanceToMoveX);
        //driveBase.drive(driveZ, driveX, Math.toRadians(desiredAngle));
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(botpose);

        // System.out.println(currentAngle);

        // if (Math.abs(distanceToMoveY) <= 0.3) {
        //     closeToTarget = true;
        // }
    }

    // @Override
    // public boolean isFinished() {

    //     // return Math.abs( currentAngle) <= 0.1d;
    //     if (Math.abs(getTx()) <= 3d && Math.abs(desiredDistanceZ) - 0.25 <= Math.abs(getZpos()) && Math.abs(getZpos()) <= Math.abs(desiredDistanceZ) + 0.5) {
    //         return true;
    //     }
    //     return false;
    // }

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

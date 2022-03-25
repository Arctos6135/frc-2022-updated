package frc.robot.commands.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.auto.PathFinder;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive forwards or backwards using trajectory. 
 */
public class DriveDistance {
    private PathFinder driveForwardsInit; 

    /**
     * Create a new drive distance autonomous command. 
     * 
     * @param drivetrain the drivetrain subsystem of the robot.
     * @param distance desired distance in meters. 
     */
    public DriveDistance(Drivetrain drivetrain, double distance) {
        this.driveForwardsInit = new PathFinder(
            drivetrain, 
            new Pose2d(0, 0, new Rotation2d(0)),
            null, 
            new Pose2d(distance, 0, new Rotation2d(0))
        );
    } 

    public RamseteCommand getAutoCommand() {
        return driveForwardsInit.getAutoCommand();
    }
}

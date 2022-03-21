package frc.robot.commands.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.auto.PathFinder;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance {
    private PathFinder driveForwardsInit; 

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

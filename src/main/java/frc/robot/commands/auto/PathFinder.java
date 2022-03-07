package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.AutoConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * A template for an autonomous drive command, with all units in meters. 
 * Path Finder ramsete commands can be placed in a Sequential Command to perform autonomous routines.
 */
public class PathFinder extends CommandBase {
    private DifferentialDriveVoltageConstraint voltageConstraint = 
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter,
            AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.kDriveKinematics, AutoConstants.maxVoltage);
            
    private TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(AutoConstants.kDriveKinematics)
        .addConstraint(voltageConstraint);
            
    private static Trajectory autoTrajectory; 
    private RamseteCommand autoCommand;
    private static Drivetrain drivetrain; 
    
    /**
     * Creates a new instance of the Path Finder autonomous drive command. 
     * 
     * @param drivetrain the driving subsystem.
     * @param startPosition the starting position of the autonomous command.
     * @param waypoints the points to pass through when driving. 
     * @param endPosition the ending position of the autonomous command.
     */
    public PathFinder(Drivetrain drivetrain, Pose2d startPosition, List<Translation2d> waypoints, Pose2d endPosition) {
        PathFinder.drivetrain = drivetrain;
        addRequirements(drivetrain);

        PathFinder.autoTrajectory = TrajectoryGenerator.generateTrajectory(
                startPosition, waypoints, endPosition, this.config);
        
        this.autoCommand = new RamseteCommand(autoTrajectory, drivetrain::getPose,
            new RamseteController(
                AutoConstants.kRamseteB,
                AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                AutoConstants.ksVolts,
                AutoConstants.kvVoltSecondsPerMeter,
                AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.kDriveKinematics, drivetrain::getWheelSpeeds,
            new PIDController(AutoConstants.kPDriveVel, 0, 0),
            new PIDController(AutoConstants.kPDriveVel, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain);
    }

    /**
     * Reset the robot to the starting position of the trajectory. 
     */
    public static Command resetInitialPosition() {
        return new InstantCommand(() -> {
            drivetrain.resetOdometry(autoTrajectory.getInitialPose());
        });
    }
     
    /**
     * Get the autonomous command as a ramsete command.
     * 
     * @return the autonomous command. 
     */
    public RamseteCommand getAutoCommand() {
        return this.autoCommand;
    }

    @Override 
    public void initialize() {

    }

    @Override 
    public void execute() {

    }

    @Override 
    public void end(boolean interrupted) {
        drivetrain.setMotors(0, 0); 
    }

    @Override 
    public boolean isFinished() {
        return autoCommand.isFinished(); 
    }
}

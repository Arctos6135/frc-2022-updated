package frc.robot.commands.auto;

import java.util.Collections;
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
import frc.robot.constants.AutoConstants;
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
    private final Drivetrain drivetrain; 

    private Pose2d startPosition = null; 
    private List<Translation2d> waypointsTranslation = null; 
    private Pose2d endPosition = null; 

    private List<Pose2d> waypointsPose = null;
    /**
     * Creates a new instance of the Path Finder autonomous drive command. 
     * 
     * @param drivetrain the driving subsystem.
     * @param startPosition the starting position of the autonomous command.
     * @param waypoints the points to pass through when driving. 
     * @param endPosition the ending position of the autonomous command.
     */
    public PathFinder(Drivetrain drivetrain, Pose2d startPosition, List<Translation2d> waypoints, Pose2d endPosition) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.startPosition = startPosition;
        this.waypointsTranslation = waypoints; 
        this.endPosition = endPosition;

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
            new PIDController(AutoConstants.kPDriveVel, AutoConstants.kDDriveVel, 0),
            new PIDController(AutoConstants.kPDriveVel, AutoConstants.kDDriveVel, 0),
            drivetrain::tankDriveVolts,
            drivetrain);
    }

    /**
     * Creates a new instance of the Path Finder autonomous drive command. 
     * 
     * @param drivetrain the robot driving subsystem. 
     * @param waypoints the points to pass through when driving, as Pose2d objects. 
     */
    public PathFinder(Drivetrain drivetrain, List<Pose2d> waypoints) {
        this.drivetrain = drivetrain; 
        addRequirements(drivetrain);

        this.waypointsPose = waypoints; 

        PathFinder.autoTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.config);

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
    public Command resetInitialPosition() {
        return new InstantCommand(() -> {
            drivetrain.resetOdometry(autoTrajectory.getInitialPose());
        });
    }

    public Pose2d getStartPosition() {
        if (this.startPosition == null && this.waypointsPose.size() > 0) {
            return this.waypointsPose.get(0); 
        }
        return this.startPosition; 
    }

    public List<Translation2d> getWaypointsTranslations() {
        return this.waypointsTranslation;
    }

    public Pose2d getEndPosition() {
        if (this.endPosition == null && this.waypointsPose.size() > 0) {
            return this.waypointsPose.get(waypointsPose.size() - 1); 
        }
        return this.endPosition;
    }

    public List<Pose2d> getWaypointsPose() {
        return this.waypointsPose;
    }
     
    /**
     * Get the autonomous command as a ramsete command.
     * 
     * @return the autonomous command. 
     */
    public RamseteCommand getAutoCommand() {
        return this.autoCommand;
    }

    public static RamseteCommand invertPathfinderCommand(PathFinder path) {
        List<Translation2d> pathTranslations = path.getWaypointsTranslations();
        Collections.reverse(pathTranslations); 

        return new PathFinder(
            path.drivetrain, 
            path.getEndPosition(), 
            pathTranslations, 
            path.getStartPosition()).getAutoCommand();
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

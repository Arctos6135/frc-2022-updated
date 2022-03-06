package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class MotionProfile extends CommandBase {
    private final Drivetrain drivetrain;
    private final Trajectory trajectory;
    private final RamseteController ramseteController;
    private final Timer timer;
    private final RamseteCommand autoCommand;

    public MotionProfile(Drivetrain drivetrain, double initialVelocity,
            double finalVelocity, List<Pose2d> checkpoints, boolean reversed) {
        this(drivetrain, initialVelocity, finalVelocity, checkpoints, reversed, null);
    }

    public MotionProfile(Drivetrain drivetrain, double initialVelocity,
            double finalVelocity, List<Pose2d> checkpoints, boolean reversed,
            List<TrajectoryConstraint> constraints) {
        this.drivetrain = drivetrain;
        this.ramseteController = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);
        this.timer = new Timer();

        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        AutoConstants.ksVolts,
                        AutoConstants.kvVoltSecondsPerMeter,
                        AutoConstants.kaVoltSecondsSquaredPerMeter),
                AutoConstants.kDriveKinematics,
                AutoConstants.maxVoltage);

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(AutoConstants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .addConstraints(constraints)
                .setStartVelocity(initialVelocity)
                .setEndVelocity(finalVelocity)
                .setReversed(reversed);

        Trajectory autoTrajectory;

        try {
            autoTrajectory = TrajectoryGenerator.generateTrajectory(checkpoints, trajectoryConfig);
        } catch (TrajectoryGenerationException exception) {
            autoTrajectory = new Trajectory();
            DriverStation.reportError("Failed to generate trajectory.", true);
        }
        trajectory = autoTrajectory;

        this.autoCommand = new RamseteCommand(
                autoTrajectory,
                drivetrain::getPose,
                ramseteController,
                new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter,
                        AutoConstants.kaVoltSecondsSquaredPerMeter),
                AutoConstants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(AutoConstants.kPDriveVel, 0, 0),
                new PIDController(AutoConstants.kPDriveVel, 0, 0),
                drivetrain::tankDriveVolts,
                drivetrain);

        this.drivetrain.resetOdometry(autoTrajectory.getInitialPose());
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        State setpoint = trajectory.sample(timer.get());
        RobotContainer.getLogger().logInfo("Setpoint X: " + Double.toString(setpoint.poseMeters.getX()));
        RobotContainer.getLogger().logInfo("Setpoint Y: " + Double.toString(setpoint.poseMeters.getY()));
        RobotContainer.getLogger()
                .logInfo("Rotation: " + Double.toString(setpoint.poseMeters.getRotation().getRadians()));
        ChassisSpeeds chassisSpeeds = ramseteController.calculate(drivetrain.getPose(), setpoint);
        DifferentialDriveWheelSpeeds wheelSpeeds = AutoConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);
        drivetrain.setMotors(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        // TODO: convert from m/s to SPARK MAX [-1.0, 1.0]
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drivetrain.setMotors(0, 0);
    }

    @Override 
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public double getElapsedTime() {
        return trajectory.getTotalTimeSeconds();
    }

    public RamseteCommand getRamseteCommand() {
        return this.autoCommand;
    }
}



package frc.robot.commands;

import java.util.function.Supplier;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.DynamicTankDriveFollower;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.follower.Follower.AdvancedPositionSource;
import com.arctos6135.robotpathfinder.follower.FollowerRunner;
import com.arctos6135.robotpathfinder.follower.SimpleFollowerRunner;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower.TankDriveGains;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower.TankDriveRobot;
import com.arctos6135.stdplug.api.datatypes.PIDVADPGains;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends CommandBase {
    
    // PID Control
    private static final PIDVADPGains gains = new PIDVADPGains(0, 0, 0, 0, 0, 0);
    private static double updateDelay = 0.25; // seconds 

    private final Drivetrain drivetrain;
    
    private final TankDriveRobot tankDriveRobot;
    
    private final Followable<TankDriveMoment> profile;
    private volatile Follower<TankDriveMoment> follower;
    private final FollowerRunner runner;
    
    /**
     * Get the set of gains used to follow trajectories. 
     * 
     * @return the PIDVADP gains.
     */
    public static PIDVADPGains getGains() {
        return gains;
    }
    
    /**
     * Get the update delay for dynamic trajectories. 
     * 
     * @return the update delay. 
     */
    public static double getUpdateDelay() {
        return updateDelay;
    }
    
    /**
     * Set the update delay for dynamic trajectories. 
     * 
     * @param updateDelay the update delay.
     */
    public static void setUpdateDelay(double updateDelay) {
        FollowTrajectory.updateDelay = updateDelay;
    }
    
    /* 
    private static TankDriveGains getRobotPathfinderGains() {
        return new TankDriveGains(gains.getV(), gains.getA(), gains.getP(), gains.getI(), gains.getD(), gains.getDP());
    } */ 

    public FollowTrajectory(Drivetrain drivetrain, Followable<TankDriveMoment> profile) {
        this.drivetrain = drivetrain;
        this.profile = profile;
        addRequirements(drivetrain);

        this.runner = new SimpleFollowerRunner();
        this.tankDriveRobot = new TankDriveRobot(drivetrain::setLeftMotor, drivetrain::setRightMotor,
                new FunctionAdvancedPositionSource(drivetrain::getLeftVelocity, drivetrain::getLeftDistance),
                new FunctionAdvancedPositionSource(drivetrain::getRightVelocity, drivetrain::getRightDistance),
                Timer::getFPGATimestamp, () -> Math.toRadians(-drivetrain.getHeading()));
    }

    @Override
    public void initialize() {
        if (profile instanceof DynamicFollowable) {
            DynamicFollowable<TankDriveMoment> dynamicProfile = (DynamicFollowable<TankDriveMoment>) profile;

            // follower = new DynamicTankDriveFollower(dynamicProfile, tankDriveRobot, getRobotPathfinderGains(),
            //        updateDelay);
        } else {
            //follower = new TankDriveFollower(profile, tankDriveRobot, getRobotPathfinderGains());
        }
        
        drivetrain.setMotorMode(IdleMode.kBrake);
        // Run the follower at 100 Hz
        runner.start(follower, 100); 
    }

    @Override
    public void execute() {

    }

    /**
     * Called when the command has ended or is interrupted. 
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotors(0, 0);
        drivetrain.setMotorMode(IdleMode.kCoast);
    }

    /**
     * Returns true when command should end.
     */
    @Override
    public boolean isFinished() {
        return runner.isFinished(); 
    }
    
    private static class FunctionAdvancedPositionSource implements AdvancedPositionSource {

        // Get the real time inputs (that often change) 
        private Supplier<Double> positionFunction;
        private Supplier<Double> velocityFunction;
        
        /**
         * Instantiate a new FunctionAdvancedPositionSource from position and velocity suppliers. 
         * Suppliers are used instead of doubles, since these values change frequently.
         * 
         * @param positionFunction the position supplier.
         * @param velocityFunction the velocity supplier. 
         */
        public FunctionAdvancedPositionSource(Supplier<Double> positionFunction, Supplier<Double> velocityFunction) {
            this.positionFunction = positionFunction;
            this.velocityFunction = velocityFunction; 
        }

        /**
         * Get the position from the position supplier. 
         * 
         * @return the double from the position supplier. 
         */
        @Override
        public double getPosition() {
            return positionFunction.get(); 
        }
        
        /**
         * Get the velocity from the velocity supplier. 
         * 
         * @return the double from the velocity supplier.
         */
        @Override
        public double getVelocity() {
            return velocityFunction.get();
        }

        @Override
        public double getAcceleration() {
            return 0;
        }
    }
}

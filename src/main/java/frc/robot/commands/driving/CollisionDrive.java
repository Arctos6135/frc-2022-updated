package frc.robot.commands.driving;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives forward until the robot collides into the fender.
 *
 * This will be done by calculating acceleration and jerk.
 */
public class CollisionDrive extends CommandBase {

    private AHRS ahrs;
    private Drivetrain drivetrain;
    private double speed;
    private boolean collisionDetected = false;

    // Acceleration
    double lastAccelX = 0;
    double lastAccelY = 0;
   
    /**
     * Create a new Collision Drive command instance.
     *
     * @param ahrs the robot navigation system.
     * @param drivetrain the robot drivetrain.
     * @param speed the speed the drivetrain will be set to.
     */
    public CollisionDrive(AHRS ahrs, Drivetrain drivetrain, double speed) {
        this.ahrs = ahrs;
        this.drivetrain = drivetrain;
        this.speed = speed;

        addRequirements(drivetrain);
    }
   
    /**
     * Sets the motors to a slow speed.
     */
    @Override
    public void initialize() {
        // Set the motors to a slow speed, since it will be colliding with the fender.
        if (Math.abs(speed) > 0.5) {
            speed = Math.copySign(0.5, speed);
            drivetrain.setMotors(speed, speed);
        } else {
            drivetrain.setMotors(speed, speed);
        }
    }

    /**
     * Repeatedly gets the acceleration from the AHRS to calculate jerk.
     *
     * When the jerk passes the collision threshold, a collision has been detected.
     */
    @Override
    public void execute() {
        double currAccelX = ahrs.getWorldLinearAccelX();
        double currAccelY = ahrs.getWorldLinearAccelY();

        double currJerkX = currAccelX - lastAccelX;
        double currJerkY = currAccelY - lastAccelY;

        lastAccelX = currAccelX;
        lastAccelY = currAccelY;

        // Detect Collision
        if ((Math.abs(currJerkX) > Constants.COLLISION_THRESHOLD)
                || (Math.abs(currJerkY) > Constants.COLLISION_THRESHOLD)) {
            collisionDetected = true;
        }
    }

    /**
     * If a collision has been detected, the command ends.
     */
    @Override
    public boolean isFinished() {
        return collisionDetected;
    }

    /**
     * Stop the motors on drivetrain when the command ends.
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotors(0, 0);
    }
}
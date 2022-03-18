package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * Shoots a ball (mostly to the low hub). 
 * When initialized, the shooter motors are set to a constant RPM depending on low or high hub. 
 * After the velocity of the shooter motors are set using PID control, the command waits until 
 * the motors reach the required RPM to start rolling the shooter feeder subsystem. 
 */
public class Shoot extends CommandBase {
    
    private static final double VELOCITY_TOLERANCE = 100; 
    
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeederSubsystem; 

    private double targetVelocity = 0; 
    private boolean velocityReached = false; 
    private boolean lowerHub; 

    private boolean finished = false; 

    /**
     * 
     * @param shooter the shooter subsystem with shooter wheels and motors.
     * @param shooterFeederSubsystem the shooter feeder subsystem with belts. 
     * @param lowerHub whether to shoot high or low hub. 
     */
    public Shoot(Shooter shooter, ShooterFeederSubsystem shooterFeederSubsystem, boolean lowerHub) {
        this.shooter = shooter; 
        this.shooterFeederSubsystem = shooterFeederSubsystem; 
        this.lowerHub = lowerHub; 

        addRequirements(shooter, shooterFeederSubsystem);

        this.shooterFeederSubsystem.setRollDirection(false);
        this.shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED); 
    }

    @Override 
    public void initialize() {
        finished = false; 
        velocityReached = false; 

        if (!shooter.getOverheatShutoffOverride() && shooter.getMonitorGroup().getOverheatShutoff()) {
            finished = true; 
            RobotContainer.getLogger().logError("Shooter is overheating, cannot shoot."); 
        }

        if (lowerHub) {
            shooter.setVelocityDirectly(0.5);
            targetVelocity = Constants.LOW_HUB_RPM;  
        } else {
            shooter.setVelocityDirectly(0.9); 
            targetVelocity = Constants.HIGH_HUB_RPM;
        }
        shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED);
    }

    @Override 
    public void execute() {
        /*
        if (Math.abs(shooter.getActualVelocity() - targetVelocity) < VELOCITY_TOLERANCE && shooterFeederSubsystem.getBallInShotPosition()) {
            // Shoot the ball 
            velocityReached = true; 
            shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED);

            /* if (lowerHub) {
                try {
                    shooter.fire(false); 
                } catch (Shooter.PowerException exception) {
                    RobotContainer.getLogger().logError("The shooter motor cannot support the lower hub shot!");
                } 
                
            } else {
                try {
                    shooter.fire(true); 
                } catch (Shooter.PowerException exception) {
                    RobotContainer.getLogger().logError("The shooter motor cannot support the upper hub shot!"); 
                } 
            } */ /*

        } else {
            shooterFeederSubsystem.stopRoller(); 

            if (velocityReached) {
                shooterFeederSubsystem.decrementBallCount(); 
            }
        } */ 

    }

    @Override 
    public void end(boolean interrupted) {
        // Stop the feeder 
        shooter.setVelocity(0);
        shooterFeederSubsystem.stopRoller(); 
    }

    @Override 
    public boolean isFinished() {
        return finished; 
    }
}

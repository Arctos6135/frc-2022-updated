package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * Shoot a ball by rolling to the shooters. 
 * The shooter velocity is set using PID control.
 * @see {@link PrepareShooterPID}
 */
public class PIDShoot extends CommandBase {
    
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeederSubsystem; 

    private double targetVelocity = 0; 
    private boolean lowerHub;

    private boolean rpmReached = false;  

    /**
     * Creates a new command for shooting with PID control.
     * 
     * @param shooter the shooter subsystem, with integrated PID controllers.
     * @param shooterFeederSubsystem the shooter feeder subsystem (rollers).
     * @param lowerHub whether to shoot low or high hub. 
     */
    public PIDShoot(Shooter shooter, ShooterFeederSubsystem shooterFeederSubsystem, boolean lowerHub) {
        this.shooter = shooter; 
        this.shooterFeederSubsystem = shooterFeederSubsystem; 
        this.lowerHub = lowerHub; 

        addRequirements(shooter, shooterFeederSubsystem);
    }

    @Override 
    public void initialize() {
        if (!shooter.getOverheatShutoffOverride() && shooter.getMonitorGroup().getOverheatShutoff()) {
            rpmReached = true; 
            RobotContainer.getLogger().logError("Shooter is overheating, cannot shoot."); 
        } 
        
        if (this.lowerHub) {
            shooter.setVelocity(Constants.LOW_HUB_RPM);
            targetVelocity = Constants.LOW_HUB_RPM;
        } else {
            shooter.setVelocity(Constants.HIGH_HUB_RPM);
            targetVelocity = Constants.HIGH_HUB_RPM;
        }
    }

    @Override 
    public void execute() { 
        DriverStation.reportWarning(Double.toString(shooter.getActualVelocity()), true); 

        if (Math.abs(shooter.getActualVelocity() - this.targetVelocity) <= Shooter.VELOCITY_TOLERANCE) {
            this.shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED);
            RobotContainer.shooterRumbleOperator.execute();
            
            this.rpmReached = true; 
        } else if (rpmReached) {
            // Shoot a second ball after a delay. 
            this.shooter.setVelocity(this.targetVelocity);
            this.shooterFeederSubsystem.setRollSpeed(0); 
        }
    }

    @Override 
    public void end(boolean interrupted) {
        // Stop all motors for shooter and shooter roller subsystems.
        this.shooterFeederSubsystem.setRollSpeed(0);
        this.shooter.setVelocity(0);
    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
}

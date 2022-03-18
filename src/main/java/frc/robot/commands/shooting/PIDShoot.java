package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

public class PIDShoot extends CommandBase {
    
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeederSubsystem; 

    private double targetVelocity = 0; 
    private boolean velocityReached = false; 
    private boolean lowerHub;

    private boolean finished = false; 

    public PIDShoot(Shooter shooter, ShooterFeederSubsystem shooterFeederSubsystem, boolean lowerHub) {
        this.shooter = shooter; 
        this.shooterFeederSubsystem = shooterFeederSubsystem; 
        this.lowerHub = lowerHub; 

        addRequirements(shooter, shooterFeederSubsystem);
    }

    @Override 
    public void initialize() {
        if (!shooter.getOverheatShutoffOverride() && shooter.getMonitorGroup().getOverheatShutoff()) {
            finished = true; 
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
        // Waiting for shooter to reach desired RPM for first time. 
        if (!velocityReached) {
            shooter.setVelocity(this.targetVelocity);
        }

        if (Math.abs(shooter.getActualVelocity() - this.targetVelocity) < Shooter.VELOCITY_TOLERANCE) {
            velocityReached = true; 
            shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED);             
        } else {
            shooterFeederSubsystem.stopRoller(); 

            // Ball was shot, command can finish.
            if (velocityReached) {
                finished = true;
            }
        }
    }

    @Override 
    public void end(boolean interrupted) {
        shooter.setVelocity(0);
        shooterFeederSubsystem.stopRoller(); 
    }

    @Override 
    public boolean isFinished() {
        return finished; 
    }
}

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/**
 * Sets the shooter's velocity with PID control. 
 * Does not roll the ball up towards to shooter.
 */
public class PrepareShooterPID extends CommandBase {
    private final Shooter shooter; 
    private static final double VELOCITY_TOLERANCE = 100; 

    private final double targetRPM;
    private boolean rpmReached; 

    /**
     * Creates a new shooter preparation command. 
     * 
     * @param shooter the shooter subsystem. 
     * @param rpm the desired velocity of the shooter.
     */
    public PrepareShooterPID(Shooter shooter, double rpm) {
        this.shooter = shooter; 
        this.targetRPM = rpm;
        this.rpmReached = false; 
        addRequirements(shooter);
    }

    @Override 
    public void initialize() {
        DriverStation.reportWarning("Shooter PID has started.", true); 

        if (!shooter.getOverheatShutoffOverride() && shooter.getMonitorGroup().getOverheatShutoff()) {
            rpmReached = true; 
            RobotContainer.getLogger().logError("Shooter is overheating, cannot shoot.");
            DriverStation.reportError("Shooter is overheating, cannot shoot.", true); 
        }

        this.shooter.setVelocity(this.targetRPM);
    }

    @Override 
    public void execute() {
        if (!rpmReached) {
            this.shooter.setVelocity(this.targetRPM);
    
            if (Math.abs(shooter.getActualVelocity() - targetRPM) < VELOCITY_TOLERANCE) {
                rpmReached = true;
                RobotContainer.shooterRumbleOperator.execute(); 
            }
        }
    }

    @Override 
    public void end(boolean interrupted) {
        // Indicate that the RPM has been reached.  
        RobotContainer.getLogger().logInfo("Shooter RPM has been reached."); 
        DriverStation.reportWarning("Shooter RPM has been reached.", true);
    } 

    @Override 
    public boolean isFinished() {
        return this.rpmReached; 
    }
}

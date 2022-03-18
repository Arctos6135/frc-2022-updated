package frc.robot.commands.shooting; 

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter; 

/**
 * Set the RPM of the shooter. 
 * @see {@link AutoShoot}
 * @see {@link Shoot}
 */
public class PrepareShooter extends CommandBase {
    private final Shooter shooter;

    private final double rpm; 
    private boolean rpmReached = false; 

    /**
     * Create a new shooter RPM setting command. 
     * 
     * @param shooter the shooter subsystem. 
     * @param lowerHub whether to shoot lower or upper hub. 
     */
    public PrepareShooter(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm; 
        addRequirements(shooter); 
    }

    @Override 
    public void initialize() {
        DriverStation.reportWarning("Shooter has started.", true); 
        if (!shooter.getOverheatShutoffOverride() && shooter.getMonitorGroup().getOverheatShutoff()) {
            rpmReached = true; 
            RobotContainer.getLogger().logError("Shooter is overheating, cannot shoot.");
            DriverStation.reportError("Shooter is overheating, cannot shoot.", false); 
        }
 
        this.shooter.setVelocityDirectly(this.rpm);
        rpmReached = true;
    }

    @Override 
    public void execute() {
    }

    @Override 
    public void end(boolean interrupted) {
        // Indicate that the RPM has been reached. 
        RobotContainer.shooterRumbleOperator.execute(); 
        RobotContainer.getLogger().logInfo("Shooter RPM has been reached."); 
        DriverStation.reportWarning("Shooter RPM has been reached.", false);
    } 

    @Override 
    public boolean isFinished() {
        return this.rpmReached; 
    }
}

package frc.robot.commands.shooting; 

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter; 

/**
 * Set the RPM of the shooter. 
 */
public class PrepareShooter extends CommandBase {
    private final Shooter shooter;
    private static final double VELOCITY_TOLERANCE = 100; 

    private double targetVelocity = 0; 
    private final boolean lowerHub; 
    private boolean rpmReached = false; 

    /**
     * Create a new shooter RPM setting command. 
     * 
     * @param shooter the shooter subsystem. 
     * @param lowerHub whether to shoot lower or upper hub. 
     */
    public PrepareShooter(Shooter shooter, boolean lowerHub) {
        this.shooter = shooter;
        this.lowerHub = lowerHub;

        addRequirements(shooter); 
    }

    @Override 
    public void initialize() {
        if (!shooter.getOverheatShutoffOverride() && shooter.getMonitorGroup().getOverheatShutoff()) {
            rpmReached = true; 
            RobotContainer.getLogger().logError("Shooter is overheating, cannot shoot.");
            DriverStation.reportError("Shooter is overheating, cannot shoot.", false); 
        }

        if (this.lowerHub) {
            shooter.setVelocity(Constants.LOW_HUB_RPM); 
            this.targetVelocity = Constants.LOW_HUB_RPM; 
        } else {
            shooter.setVelocity(Constants.HIGH_HUB_RPM); 
            this.targetVelocity = Constants.HIGH_HUB_RPM;
        }
    }

    @Override 
    public void execute() {
        if (Math.abs(shooter.getActualVelocity() - targetVelocity) < VELOCITY_TOLERANCE) {
            rpmReached = true; 
        }
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

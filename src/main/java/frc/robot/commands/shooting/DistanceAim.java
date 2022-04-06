package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class DistanceAim extends CommandBase {
    private final Shooter shooter; 

    public static boolean shooterDistance; 

    /**
     * Create a new Limelight aiming command. 
     * 
     * @param shooter the shooting subsystem. 
     */
    public DistanceAim(Shooter shooter) {
        this.shooter = shooter; 
    }

    @Override 
    public void initialize() {

    }

    @Override 
    public void execute() {
        /* double distance = shooter.getLimelight().estimateDistance(
            Constants.LIMELIGHT_HEIGHT, Constants.TARGET_HEIGHT, Constants.LIMELIGHT_ANGLE);
        
        if (Math.abs(Constants.TARGET_DISTANCE - distance) <= Constants.TARGET_DISTANCE_TOLERANCE) {
            DistanceAim.shooterDistance = true; 
        } else {
            DistanceAim.shooterDistance = false; 
        } */ 
    }

    @Override 
    public void end(boolean interrupted) {

    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
}

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class DistanceAim extends CommandBase {
    private final Shooter shooter; 

    public static boolean shooterRightDistance; 
    public static double shotDistance;
    public static boolean limelightDetected;

    /**
     * Create a new Limelight aiming command. 
     * 
     * @param shooter the shooting subsystem. 
     */
    public DistanceAim(Shooter shooter) {
        this.shooter = shooter; 
        addRequirements(shooter);
    }

    @Override 
    public void initialize() {
        DistanceAim.limelightDetected = shooter.getLimelight().hasValidTargets(); 
        DistanceAim.shotDistance = shooter.getLimelight().estimateDistance(
            Constants.LIMELIGHT_HEIGHT, Constants.TARGET_HEIGHT, Constants.LIMELIGHT_ANGLE
        );
    }

    @Override 
    public void execute() {
        if (shooter.getLimelight().hasValidTargets()) {
            DistanceAim.limelightDetected = true; 
            
            double distance = shooter.getLimelight().estimateDistance(
                Constants.LIMELIGHT_HEIGHT, Constants.TARGET_HEIGHT, Constants.LIMELIGHT_ANGLE
            );

            DistanceAim.shotDistance = distance;
        
            if (Math.abs(Constants.TARGET_DISTANCE - distance) <= Constants.TARGET_DISTANCE_TOLERANCE) {
                DistanceAim.shooterRightDistance = true; 
            } else {
                DistanceAim.shooterRightDistance = false; 
            } 
        }  
    }

    @Override 
    public void end(boolean interrupted) {

    }

    @Override 
    public boolean isFinished() {
        return false; 
    }

    public static boolean getLimelightDetected() {
        return DistanceAim.limelightDetected;
    }

    public static double getShotDistance() {
        return DistanceAim.shotDistance; 
    }

    public static boolean getShooterRightDistance() {
        return DistanceAim.shooterRightDistance;
    }
}

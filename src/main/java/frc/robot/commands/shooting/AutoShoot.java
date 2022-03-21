package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * After intaking a ball in autonomous, roll shooter feeder belts up to 
 * reach the shooter. 
 */
public class AutoShoot extends CommandBase {
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeederSubsystem; 

    private double targetVelocity = 0; 
    private boolean velocityReached = false; 
    private boolean lowerHub; 
    private double numBallsLeft;

    private boolean finished = false; 

    /**
     * Creates a new autonomous shooting command.
     * 
     * @param shooter the shooter subsystem with shooter wheels and motors.
     * @param shooterFeederSubsystem the shooter feeder subsystem with belts. 
     * @param lowerHub whether to shoot high or low hub. 
     * @param numBalls the number of balls to be shot.
     */
    public AutoShoot(Shooter shooter, ShooterFeederSubsystem shooterFeederSubsystem, boolean lowerHub, double numBalls) {
        this.shooter = shooter; 
        this.shooterFeederSubsystem = shooterFeederSubsystem; 
        this.lowerHub = lowerHub; 
        this.numBallsLeft = numBalls; 

        addRequirements(shooter, shooterFeederSubsystem);

        this.shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED); 
    }

    @Override 
    public void initialize() {
        if (!shooter.getOverheatShutoffOverride() && shooter.getMonitorGroup().getOverheatShutoff()) {
            finished = true; 
            RobotContainer.getLogger().logError("Shooter is overheating, cannot shoot.");
            DriverStation.reportError("Shooter is overheating, cannot shoot.", false); 
        }

        if (lowerHub) {
            shooter.setVelocity(Constants.LOW_HUB_RPM); 
            this.targetVelocity = Constants.LOW_HUB_RPM;
        } else {
            shooter.setVelocity(Constants.HIGH_HUB_RPM); 
            this.targetVelocity = Constants.HIGH_HUB_RPM;
        }
    }

    @Override 
    public void execute() {
        if (Math.abs(shooter.getActualVelocity() - targetVelocity) < Shooter.VELOCITY_TOLERANCE) {
            velocityReached = true; 
            shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED);
        } else {
            // Ball has been shot.
            if (velocityReached) {
                this.numBallsLeft--;
            }

            // No balls left to shoot.
            if (this.numBallsLeft <= 0) {
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
        return finished || this.numBallsLeft <= 0;
    }
    
}

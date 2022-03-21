package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * From the top of the shooter feeder subsystem, 
 * roll balls down for shooting. 
 */
public class AutoLoad extends CommandBase {
    private final ShooterFeederSubsystem shooterFeeder; 

    private boolean finished; 

    public static double loadTime = 0.5; // seconds 
    private double initialTime; 

    public AutoLoad(ShooterFeederSubsystem shooterFeeder) {
        this.shooterFeeder = shooterFeeder; 
        addRequirements(shooterFeeder);
    }

    @Override 
    public void initialize() {
        this.initialTime = Timer.getFPGATimestamp(); 
        shooterFeeder.setRollSpeed(-Constants.ROLL_SPEED);
    }

    @Override 
    public void execute() {
        double time = Timer.getFPGATimestamp(); 

        if (Math.abs(time - this.initialTime) >= this.initialTime) {
            finished = true; 
        }
    }

    @Override 
    public void end(boolean interrupted) {
        shooterFeeder.setRollSpeed(0);
    }

    @Override 
    public boolean isFinished() {
        return finished; 
    }
}


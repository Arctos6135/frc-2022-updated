package frc.robot.commands.indexer;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * Roll a ball from halfway up the shooter roller to the shooter wheels. 
 */
public class AutoFeed extends CommandBase {
    private final ShooterFeederSubsystem shooterFeederSubsystem; 

    private ColorMatch colorMatch; 
    private Color detectedColor; 
    private ColorMatchResult matchedColor; 

    private boolean finished;
    private double initialTime;
    public static final double autoFeedTime = 2.5; 

    public AutoFeed(ShooterFeederSubsystem shooterFeederSubsystem) {
        this.shooterFeederSubsystem = shooterFeederSubsystem;
        this.colorMatch = new ColorMatch(); 
        this.colorMatch.setConfidenceThreshold(0.80);

        this.finished = false; 

        addRequirements(shooterFeederSubsystem);

        colorMatch.addColorMatch(Color.kBlue);
        colorMatch.addColorMatch(Color.kRed); 
        colorMatch.addColorMatch(Color.kOrange); 
    }

    @Override 
    public void initialize() {
        detectedColor = this.shooterFeederSubsystem.getColorDetected(); 
        matchedColor = colorMatch.matchClosestColor(detectedColor); 

        if (matchedColor.color == Constants.OUR_ALLIANCE) {
            shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED);
            initialTime = Timer.getFPGATimestamp();
        } else {
            finished = true; 
        }
    }

    @Override 
    public void execute() {
        double time = Timer.getFPGATimestamp(); 

        if (Math.abs(time - initialTime) >= AutoFeed.autoFeedTime) {
            finished = true; 
        }
    }

    @Override 
    public void end(boolean interrupted) {
        shooterFeederSubsystem.setRollSpeed(0);
    }

    @Override 
    public boolean isFinished() {
        return finished;
    }
}

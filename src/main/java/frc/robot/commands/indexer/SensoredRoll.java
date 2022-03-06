package frc.robot.commands.indexer;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * Using the Shooter Feeder mechanism, roll a ball up the shaft using the feeder belts until 
 * the ball is sensed by the color sensor. The compression of the ball should keep it from falling.
 * 
 * Note that this command is a rough sketch of how the actual command will function.
 */
public class SensoredRoll extends CommandBase {
    
    private final ShooterFeederSubsystem shooterFeederSubsystem; 
    private ColorMatch colorMatch; 
    private Color detectedColor;
    private ColorMatchResult matchedColor; 

    public SensoredRoll(ShooterFeederSubsystem shooterFeederSubsystem) {
        this.shooterFeederSubsystem = shooterFeederSubsystem; 
        addRequirements(shooterFeederSubsystem);

        colorMatch.addColorMatch(Color.kBlue); 
        colorMatch.addColorMatch(Color.kRed); 
        colorMatch.addColorMatch(Color.kWhite); 
        // TODO: add color of whatever the color sensor is facing (for now it is white)
    }

    @Override 
    public void initialize() {
        shooterFeederSubsystem.setRollDirection(true);
        shooterFeederSubsystem.startRoller(); 
    }

    @Override 
    public void execute() {
        detectedColor = this.shooterFeederSubsystem.getColorDetected(); 
        matchedColor = colorMatch.matchClosestColor(detectedColor); 
        
        // Assumes that we are in allicance BLUE 
        if (matchedColor.color == Constants.OUR_ALLIANCE) {
            // Shoot the ball 
            shooterFeederSubsystem.setBallInShotPosition(true);
            shooterFeederSubsystem.incrementBallCount();
        } else if (matchedColor.color == Constants.OPPOSING_ALLIANCE) {
            // Outtake the ball 
            shooterFeederSubsystem.setRollDirection(false);
            shooterFeederSubsystem.startRoller(); 
        } else {
            shooterFeederSubsystem.setBallInShotPosition(false); 
        }
    }

    @Override 
    public void end(boolean interrupted) {
        shooterFeederSubsystem.stopRoller(); 
    }

    @Override 
    public boolean isFinished() { 
        return this.shooterFeederSubsystem.getBallInShotPosition(); 
    }
}

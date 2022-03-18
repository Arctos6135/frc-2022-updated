package frc.robot.commands.indexer;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
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

    // If a ball is already in position, this boolean allows the command to run. 
    private boolean ballInPosition;

    public SensoredRoll(ShooterFeederSubsystem shooterFeederSubsystem) {
        this.shooterFeederSubsystem = shooterFeederSubsystem; 
        this.colorMatch = new ColorMatch();
        this.colorMatch.setConfidenceThreshold(0.75);

        this.ballInPosition = false;
        addRequirements(shooterFeederSubsystem);
        // TODO: add color of whatever the color sensor is facing (for now it is white)
    }

    @Override 
    public void initialize() {
        colorMatch.addColorMatch(Color.kBlue); 
        colorMatch.addColorMatch(Color.kRed); 
        colorMatch.addColorMatch(Color.kWhite);  

        shooterFeederSubsystem.setRollDirection(true);
        shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED_SENSING);
    }

    @Override 
    public void execute() {
        detectedColor = this.shooterFeederSubsystem.getColorDetected(); 
        matchedColor = colorMatch.matchClosestColor(detectedColor); 
        
        DriverStation.reportWarning(Double.toString(detectedColor.blue), true); 
        DriverStation.reportWarning(Double.toString(detectedColor.red), true);
        
        // Assumes that we are in allicance BLUE 
        if (matchedColor.color == Constants.OUR_ALLIANCE) {
            // Store the ball halfway up the belts
            shooterFeederSubsystem.setBallInShotPosition(true);
            shooterFeederSubsystem.incrementBallCount();
            this.ballInPosition = true;
            shooterFeederSubsystem.stopRoller();
        } else if (matchedColor.color == Constants.OPPOSING_ALLIANCE) {
            // Outtake the ball 
            shooterFeederSubsystem.setRollSpeed(-Constants.ROLL_SPEED_SENSING); 
        } else {
            shooterFeederSubsystem.setBallInShotPosition(false); 
        } 
    }

    @Override 
    public void end(boolean interrupted) {
        shooterFeederSubsystem.setRollDirection(false);
        shooterFeederSubsystem.stopRoller(); 
        shooterFeederSubsystem.setBallInShotPosition(false);
        DriverStation.reportWarning("Sensored Roll Command Ended.", true); 
    }

    @Override 
    public boolean isFinished() { 
        return this.shooterFeederSubsystem.getBallInShotPosition() && ballInPosition; 
    }
}

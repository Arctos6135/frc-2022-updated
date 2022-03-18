package frc.robot.commands.indexer;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
    private boolean ballAtColorSensor;
    
    private boolean outtake;
    private boolean outtakeFinished; 
    
    // Functionality for Outtaking Balls
    private final Timer timer; 
    private double initialOuttakeTime; 

    /**
     * Creates a new Teleoperated ball rolling command. 
     * 
     * @param shooterFeederSubsystem the subsystem with the belt rollers. 
     */
    public SensoredRoll(ShooterFeederSubsystem shooterFeederSubsystem) {
        this.shooterFeederSubsystem = shooterFeederSubsystem; 
        this.colorMatch = new ColorMatch();
        this.colorMatch.setConfidenceThreshold(0.80);
        addRequirements(shooterFeederSubsystem);

        this.timer = new Timer(); 

        colorMatch.addColorMatch(Color.kBlue); 
        colorMatch.addColorMatch(Color.kRed); 
        colorMatch.addColorMatch(Color.kBlack); // TODO: put something in front of color sensor
    }

    @Override 
    public void initialize() {
        shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED);
        this.outtake = false; 
        this.outtakeFinished = false; 
        this.ballAtColorSensor = false; 
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
            this.ballAtColorSensor = true;
            // shooterFeederSubsystem.stopRoller();
        } else if (matchedColor.color == Constants.OPPOSING_ALLIANCE) {
            // Outtake the ball 
            this.outtake = true; 
            this.initialOuttakeTime = this.timer.get(); 
            shooterFeederSubsystem.setRollSpeed(-Constants.ROLL_SPEED); 
        } else if (this.outtake) {
            shooterFeederSubsystem.setRollSpeed(-Constants.ROLL_SPEED);

            if (this.timer.get() - this.initialOuttakeTime >= Constants.OUTTAKE_TIME) {
                this.outtakeFinished = true; 
            }
        }
        else {
            shooterFeederSubsystem.setBallInShotPosition(false);
            shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED); 
        } 
    }

    @Override 
    public void end(boolean interrupted) {
        shooterFeederSubsystem.stopRoller(); 
        shooterFeederSubsystem.setBallInShotPosition(false);
        DriverStation.reportWarning("Sensored Roll Command Ended.", true); 
    }

    @Override 
    public boolean isFinished() { 
        // Wrong colored ball.
        if (this.outtake) {
            return this.outtakeFinished; 
        } 
        return this.shooterFeederSubsystem.getBallInShotPosition() && this.ballAtColorSensor; 
    }
}

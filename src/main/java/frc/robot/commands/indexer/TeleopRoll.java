package frc.robot.commands.indexer;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * Rolls the ball at a speed and direction based on the joystick movement.
 */
public class TeleopRoll extends CommandBase {
    
    private final ShooterFeederSubsystem shooterFeederSubsystem; 
    private final XboxController operatorController;
    private final int rollUpButton;
    private final int rollDownButton;  

    /* private ColorMatch colorMatch; 
    private Color detectedColor; 
    private ColorMatchResult matchedColor; */ 

    public static double rollPrecisionFactor = 0.5; 

    /**
     * Creates a new instance of the roll command. 
     * 
     * @param shooterFeederSubsystem the shooter feeder subsystem with the roller motor. 
     * @param operatorController the controller interacting with the shooter feeder subsystem. 
     */
    public TeleopRoll(ShooterFeederSubsystem shooterFeederSubsystem, XboxController operatorController, 
        int rollUpButton, int rollDownButton) {
        this.shooterFeederSubsystem = shooterFeederSubsystem; 
        this.operatorController = operatorController;
        this.rollUpButton = rollUpButton;
        this.rollDownButton = rollDownButton;  

        addRequirements(shooterFeederSubsystem);

        /* colorMatch.addColorMatch(Constants.OUR_ALLIANCE); 
        colorMatch.addColorMatch(Constants.OPPOSING_ALLIANCE); 
        colorMatch.addColorMatch(Color.kWhite);  */ 
        // TODO: change to whatever the color of the board of the color sensor
    }

    @Override 
    public void execute() {
        double rollSpeedUp = operatorController.getRawButton(this.rollUpButton) ? Constants.ROLL_SPEED : 0; 
        double rollSpeedDown = operatorController.getRawButton(this.rollDownButton) ? Constants.ROLL_SPEED : 0; 

        if (rollSpeedUp == Constants.ROLL_SPEED && rollSpeedDown == 0) {
            shooterFeederSubsystem.setRollSpeed(-rollSpeedUp); 
        } else if (rollSpeedUp == 0 && rollSpeedDown == Constants.ROLL_SPEED) {
            shooterFeederSubsystem.setRollSpeed(rollSpeedDown);
        } else {
            shooterFeederSubsystem.setRollSpeed(0);
        }

        /*
        detectedColor = this.shooterFeederSubsystem.getColorDetected(); 
        matchedColor = colorMatch.matchColor(detectedColor); 

        if (matchedColor.color == Constants.OUR_ALLIANCE) {
            RobotContainer.getLogger().logInfo("Alliance ball in indexer.");
            DriverStation.reportWarning("Alliance ball in indexer.", false);
            shooterFeederSubsystem.incrementBallCount();
            shooterFeederSubsystem.setBallInShotPosition(true);
        } 
        else if (matchedColor.color == Constants.OPPOSING_ALLIANCE) {
            RobotContainer.getLogger().logInfo("Wrong ball in indexer."); 
            DriverStation.reportWarning("Wrong ball in indexer.", false); 
            shooterFeederSubsystem.setBallInShotPosition(false);
        } */ 
    }

    @Override 
    public void end(boolean interrupted) {
        shooterFeederSubsystem.stopRoller(); 
    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
}

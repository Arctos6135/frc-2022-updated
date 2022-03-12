package frc.robot.commands.indexer;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * Rolls the ball at a speed and direction based on the joystick movement.
 */
public class TeleopRoll extends CommandBase {
    
    private final ShooterFeederSubsystem shooterFeederSubsystem; 
    private final XboxController operatorController;
    private final int Y_AXIS; 

    private ColorMatch colorMatch; 
    private Color detectedColor; 
    private ColorMatchResult matchedColor; 

    /**
     * Creates a new instance of the roll command. 
     * 
     * @param shooterFeederSubsystem the shooter feeder subsystem with the roller motor. 
     * @param operatorController the controller interacting with the shooter feeder subsystem. 
     */
    public TeleopRoll(ShooterFeederSubsystem shooterFeederSubsystem, XboxController operatorController, int rollAxis) {
        this.shooterFeederSubsystem = shooterFeederSubsystem; 
        this.operatorController = operatorController;
        this.Y_AXIS = rollAxis; 

        addRequirements(shooterFeederSubsystem);

        colorMatch.addColorMatch(Color.kBlue); 
        colorMatch.addColorMatch(Color.kRed); 
        colorMatch.addColorMatch(Color.kWhite);  
        // TODO: change to whatever the color of the board of the color sensor
    }

    @Override 
    public void execute() {
        if (ShooterFeederSubsystem.constantRollSpeed) {
            shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED); 
        } else {
            double rollSpeed = TeleopDrive.applyDeadband(operatorController.getRawAxis(Y_AXIS), Constants.CONTROLLER_DEADZONE);
            rollSpeed = Math.copySign(rollSpeed * rollSpeed, rollSpeed); 

            shooterFeederSubsystem.setRollSpeed(rollSpeed);
        }

        detectedColor = this.shooterFeederSubsystem.getColorDetected(); 
        matchedColor = colorMatch.matchColor(detectedColor); 

        if (matchedColor.color == Constants.OUR_ALLIANCE) {
            RobotContainer.getLogger().logInfo("Blue ball in indexer (correct color).");
        } 
        else if (matchedColor.color == Constants.OPPOSING_ALLIANCE) {
            RobotContainer.getLogger().logInfo("Red ball in indexer (wrong color)."); 
        }
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

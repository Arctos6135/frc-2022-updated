package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    }

    @Override 
    public void execute() {
        boolean rollSpeedUp = operatorController.getRawButton(this.rollUpButton); 
        boolean rollSpeedDown = operatorController.getRawButton(this.rollDownButton); 

        if (rollSpeedUp && !rollSpeedDown) {
            shooterFeederSubsystem.setRollSpeed(Constants.ROLL_SPEED); 
        } else if (!rollSpeedUp && rollSpeedDown) {
            shooterFeederSubsystem.setRollSpeed(-Constants.ROLL_SPEED);
        } else {
            shooterFeederSubsystem.setRollSpeed(0);
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

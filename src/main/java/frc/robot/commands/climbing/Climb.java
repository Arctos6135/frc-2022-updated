package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * Pulls the robot up using two motors on the climb subsystem to roll the winch.    
 */
public class Climb extends CommandBase {
    
    private final ClimbSubsystem climbSubsystem; 
    private final XboxController operatorController;
    private final int deployClimb; 

    private static boolean overrideTime; 

    /**
     * Create an instance of the climb command. 
     * 
     * @param climbSubsystem the climb subsystem with the climb and hook motors. 
     * @param operatorController the controller used to interact with the climb commands. 
     */
    public Climb(ClimbSubsystem climbSubsystem, XboxController operatorController, int deployClimb) {
        this.climbSubsystem = climbSubsystem; 
        this.operatorController = operatorController;
        this.deployClimb = deployClimb;

        addRequirements(climbSubsystem);
    }

    public static void toggleOverride() {
        overrideTime = !overrideTime; 
    }

    @Override 
    public void initialize() {

    }

    @Override 
    public void execute() {
        double climbSpeed = operatorController.getRawButton(deployClimb) ? 0.5 : 0.0;

        // Nearing the end of the match, climb system is activated. 
        if (!overrideTime && DriverStation.getMatchTime() <= Constants.START_CLIMB_TIME) {
            this.climbSubsystem.setClimbMotorSpeed(climbSpeed);
        } 
        // Bring climb system down. 
        else if (!overrideTime && climbSpeed < 0) {
            this.climbSubsystem.setClimbMotorSpeed(climbSpeed);
        }
        // Override climb time. 
        else if (overrideTime) {
            this.climbSubsystem.setClimbMotorSpeed(climbSpeed);
        } 
        else {
            RobotContainer.getLogger().logInfo("Cannot activate climb susbsystem."); 
        }
    }

    @Override 
    public void end(boolean interrupted) {
        this.climbSubsystem.stopClimbMotors();
    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
}

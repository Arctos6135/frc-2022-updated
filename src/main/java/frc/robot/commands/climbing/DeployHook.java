package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HookSubsystem;

/**
 * Deploy the hook for climbing. The hook can be deployed and retracted. 
 */
public class DeployHook extends CommandBase {
    
    private final HookSubsystem hookSubsystem; 
    private final XboxController driverController; 

    public static final double hookDeploymentSpeed = 0.75;
    public static final double hookRetractionSpeed = -0.75; 

    /**
     * Creates a new hook deployment command. 
     * 
     * @param climbSubsystem the robot climb mechanism. 
     * @param driverController the driver controller (0). 
     */
    public DeployHook(HookSubsystem hookSubsystem, XboxController driverController) {
        this.hookSubsystem = hookSubsystem;
        this.driverController = driverController; 
        
        addRequirements(hookSubsystem);
    }
    
    @Override 
    public void initialize() {

    }

    @Override 
    public void execute() {
        boolean deployHook = driverController.getRawButton(Constants.DEPLOY_CLIMB_HOOK);
        boolean retractHook = driverController.getRawButton(Constants.RETRACT_CLIMB_HOOK); 

        if (!DriverStation.isAutonomous()) {
            if (DriverStation.getMatchTime() <= Constants.START_CLIMB_TIME || ClimbSubsystem.getClimbTimeOverride()) {
                if (deployHook) {
                    this.hookSubsystem.setHookMotorSpeed(hookDeploymentSpeed);
                } else if (retractHook) {
                    this.hookSubsystem.setHookMotorSpeed(hookRetractionSpeed); 
                } else {
                    this.hookSubsystem.setHookMotorSpeed(0);
                }
            }
        }
    }

    @Override 
    public void end(boolean interrupted) {
        this.hookSubsystem.stopHookMotor();
    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
}

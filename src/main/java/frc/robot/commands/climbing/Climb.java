package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;

/**
 * Pulls the robot up using two motors on the climb subsystem to roll the winch.    
 * This is the default command for 
 * @see Elevator.java 
 */
public class Climb extends CommandBase {
    
    private final Elevator climbSubsystem; 
    private final XboxController operatorController;
    private final int deployClimbAxis; 

    public static boolean overrideTime = false; 
    public static boolean precisionClimb = false; 
    public static double normalPrecision = 0.5; 
    public static double precisionFactor = 0.25; 

    /**
     * Create an instance of the climb command. 
     * 
     * @param climbSubsystem the climb subsystem with the climb and hook motors. 
     * @param operatorController the controller used to interact with the climb commands. 
     */
    public Climb(Elevator climbSubsystem, XboxController operatorController, int deployClimbAxis) {
        this.climbSubsystem = climbSubsystem; 
        this.operatorController = operatorController;
        this.deployClimbAxis = deployClimbAxis;

        addRequirements(climbSubsystem);
    }

    /**
     * Toggle whether to override the time restriction on the climb command.
     */
    public static void toggleTimeOverride() {
        Climb.overrideTime = !Climb.overrideTime; 
    }

    /**
     * Toggle whether to enter precision mode for climbing. 
     */
    public static void togglePrecisionClimb() {
        Climb.precisionClimb = !Climb.precisionClimb;
    }

    /**
     * Get whether climbing is in precision mode. 
     * 
     * @return whether the climb is in precision mode.
     */
    public static boolean isPrecisionClimb() {
        return Climb.precisionClimb;
    }

    @Override 
    public void initialize() {

    }

    @Override 
    public void execute() {
        double climbSpeed = operatorController.getRawAxis(this.deployClimbAxis);

        climbSpeed = TeleopDrive.applyDeadband(climbSpeed, Constants.CONTROLLER_DEADZONE);

        if (!DriverStation.isAutonomous()) {
            // Nearing the end of the match, climb system is activated. 
            if (!overrideTime && DriverStation.getMatchTime() <= Constants.START_CLIMB_TIME) {
                this.climbSubsystem.setClimbMotorSpeed(climbSpeed);
            } 
            // Override climb time. 
            else if (overrideTime) {
                this.climbSubsystem.setClimbMotorSpeed(climbSpeed);
            } 
            else {
                RobotContainer.getLogger().logInfo("Cannot activate climb subsystem."); 
            }
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
